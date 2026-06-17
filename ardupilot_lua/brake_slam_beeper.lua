-- Optional ArduPilot Lua backup beeper for the Jetson SLAM calibration monitor.
-- Install on the flight controller SD card as:
--   APM/scripts/brake_slam_beeper.lua
--
-- Jetson remains the calibration brain and owns the primary PLAY_TUNE beeps.
-- Obstacle audio is armed-only in the Jetson LiDAR node; normal SLAM/status
-- beeps remain available before arming.
-- This Lua helper is a quiet backup relay: it explains SCR_USER1 state changes
-- in GCS and only plays local tunes for FC-side events/errors, avoiding duplicate
-- startup/ready/calibration tunes from both Jetson and Cube.

local STATUS_PARAM = "SCR_USER1"
local SOURCE_SET_PARAM = "SCR_USER2"
local HEARTBEAT_PARAM = "SCR_USER3"
local OBSTACLE_STATUS_PARAM = "SCR_USER4"
local UPDATE_MS = 500
local STALE_MS = 15000
local POSHOLD_REPORT_MS = 10000
local BRAKE_REPORT_MS = 5000
local OBSTACLE_REPORT_MS = 10000

local STATE_IDLE = 0
local STATE_JETSON_BOOT = 10
local STATE_SENSOR_CHECK_PASSED = 12
local STATE_GPS_ASSIST_ACTIVE = 20
local STATE_ORIGIN_LOCKED = 30
local STATE_SLAM_STARTED = 40
local STATE_SOURCE_SET_ACTIVE = 50
local STATE_GPS_SOURCE_ACTIVE = 51
local STATE_NO_GPS_SOURCE_ACTIVE = 52
local STATE_GPS_LESS_FLIGHT_ACTIVE = 53
local STATE_POSHOLD_READY = 54
local STATE_MANUAL_ORIGIN = 60
local STATE_CALIBRATION_WAITING_ARM = 68
local STATE_CALIBRATION_WAITING_TAKEOFF = 69
local STATE_CALIBRATION_ACTIVE = 70
local STATE_CALIBRATION_COMPLETE_RTL = 71
local STATE_SLAM_FLIGHT_ACTIVE = 72
local STATE_SOURCE_SWITCH_FAILED = 82
local STATE_SOURCE_SWITCH_NO_ACK = 83

local OBSTACLE_STATUS_NONE = 0
local OBSTACLE_STATUS_NATIVE_ACTIVE = 10
local OBSTACLE_STATUS_MONITOR = 12
local OBSTACLE_STATUS_WARNING = 20
local OBSTACLE_STATUS_KEEP_OUT = 30
local OBSTACLE_STATUS_CRITICAL = 40
local OBSTACLE_STATUS_STALE = 90

local last_state = nil
local last_source_set = nil
local last_obstacle_status = nil
local last_mode = nil
local last_armed = nil
local last_heartbeat = nil
local last_heartbeat_seen_ms = 0
local heartbeat_seen_change = false
local stale_announced = false
local last_active_beep_ms = 0
local last_mode_truth_ms = 0
local last_obstacle_report_ms = 0

local MODE_ALTHOLD = 2
local MODE_LOITER = 5
local MODE_POSHOLD = 16
local MODE_BRAKE = 17

local function round_param(value)
    if value == nil then
        return 0
    end
    return math.floor(value + 0.5)
end

local function notice(text)
    gcs:send_text(5, text)
end

local function warn(text)
    gcs:send_text(4, text)
end

local function play(tune)
    if notify ~= nil and notify.play_tune ~= nil then
        notify:play_tune(tune)
    end
end

local function bridge_fresh(now_ms)
    return heartbeat_seen_change and (now_ms - last_heartbeat_seen_ms) <= STALE_MS
end

local function is_no_gps_flight_state(state)
    return state == STATE_GPS_LESS_FLIGHT_ACTIVE or state == STATE_SLAM_FLIGHT_ACTIVE
end

local function mode_name(mode)
    if mode == MODE_ALTHOLD then
        return "ALTHOLD"
    elseif mode == MODE_LOITER then
        return "LOITER"
    elseif mode == MODE_POSHOLD then
        return "POSHOLD"
    elseif mode == MODE_BRAKE then
        return "BRAKE"
    end
    return string.format("mode=%d", mode)
end

local function report_mode_truth(mode, state, fresh)
    if mode == MODE_LOITER then
        notice("LOITER active: normal GPS flight; SLAM observer should observe only.")
    elseif mode == MODE_BRAKE then
        if state == STATE_CALIBRATION_ACTIVE and fresh then
            notice("BRAKE active: SLAM calibration running.")
        elseif state == STATE_CALIBRATION_COMPLETE_RTL and fresh then
            notice("BRAKE calibration complete: profile saved.")
        elseif fresh then
            notice("BRAKE selected: waiting for Jetson calibration state.")
        else
            warn("BRAKE selected: Jetson calibration not confirmed yet.")
        end
    elseif mode == MODE_POSHOLD then
        if is_no_gps_flight_state(state) and fresh then
            notice("GPS-DENIED ACTIVE: POSHOLD using SLAM/GPS2.")
        elseif state == STATE_NO_GPS_SOURCE_ACTIVE and fresh then
            notice("POSHOLD: no-GPS source selected, waiting active confirmation.")
        elseif state == STATE_POSHOLD_READY and fresh then
            notice("POSHOLD entered: SLAM gate ready, waiting active confirmation.")
        else
            warn("POSHOLD GPS-assisted: no confirmed SLAM no-GPS state.")
        end
    else
        notice(string.format("%s active.", mode_name(mode)))
    end
end

local function report_obstacle_status(status)
    if status == OBSTACLE_STATUS_NATIVE_ACTIVE then
        notice("OA ACTIVE: FC native avoidance receiving LiDAR.")
    elseif status == OBSTACLE_STATUS_MONITOR then
        notice("OA MONITOR: LiDAR scan; FC avoidance off.")
    elseif status == OBSTACLE_STATUS_WARNING then
        warn("OA WARNING: obstacle inside warning range.")
    elseif status == OBSTACLE_STATUS_KEEP_OUT then
        warn("OA KEEP OUT: obstacle inside 1.5m.")
    elseif status == OBSTACLE_STATUS_CRITICAL then
        warn("OA CRITICAL: obstacle near 0.5m.")
    elseif status == OBSTACLE_STATUS_STALE then
        warn("OA STALE: LiDAR obstacle data not fresh.")
    end
end

local function on_state(state, source_set)
    if state == STATE_IDLE then
        notice("SLAM bridge idle: monitoring only, no SLAM control active.")
    elseif state == STATE_JETSON_BOOT then
        notice("SLAM bridge boot state: waiting for Jetson 45s startup beep.")
    elseif state == STATE_SENSOR_CHECK_PASSED then
        notice("SLAM sensor quick check passed: not full flight readiness.")
    elseif state == STATE_GPS_ASSIST_ACTIVE then
        notice("SLAM bridge GPS assist active: waiting for GPS home/origin.")
    elseif state == STATE_ORIGIN_LOCKED then
        notice("SLAM bridge origin locked from GPS/home.")
    elseif state == STATE_SLAM_STARTED then
        notice("SLAM pose stream healthy: monitoring until gate requests GPS2/PosHold.")
    elseif state == STATE_SOURCE_SET_ACTIVE then
        notice(string.format("EKF source set active: %d", source_set))
    elseif state == STATE_GPS_SOURCE_ACTIVE then
        notice(string.format("GPS source set active: %d", source_set))
    elseif state == STATE_NO_GPS_SOURCE_ACTIVE then
        notice(string.format("No-GPS source set active: %d", source_set))
    elseif state == STATE_GPS_LESS_FLIGHT_ACTIVE then
        notice("GPS-DENIED ACTIVE: POSHOLD using SLAM/GPS2")
    elseif state == STATE_POSHOLD_READY then
        notice("NO-GPS POSHOLD gate ready: check Jetson GCS details.")
    elseif state == STATE_MANUAL_ORIGIN then
        notice("SLAM bridge manual origin set; waiting for flow/GPS2 stream.")
    elseif state == STATE_CALIBRATION_WAITING_ARM then
        notice("Brake mode detected. Waiting for arm to start SLAM calibration.")
    elseif state == STATE_CALIBRATION_WAITING_TAKEOFF then
        notice("Brake calibration waiting for takeoff or 5m rangefinder height.")
    elseif state == STATE_CALIBRATION_ACTIVE then
        notice("BRAKE calibration active: hold height, pilot override ready.")
    elseif state == STATE_CALIBRATION_COMPLETE_RTL then
        notice("Calibration successful: profile saved; RTL may be requested.")
    elseif state == STATE_SLAM_FLIGHT_ACTIVE then
        notice("GPS-DENIED ACTIVE: SLAM flight confirmed.")
    elseif state == STATE_SOURCE_SWITCH_FAILED then
        warn("BEEP: EKF source switch failed")
        play("MFT160L8CBA") -- Descending scale: Error/Failure
    elseif state == STATE_SOURCE_SWITCH_NO_ACK then
        warn("BEEP: EKF source switch no ack")
        play("MFT160L8CBA") -- Descending scale: Error/Failure
    end
end

local function update()
    local mode = vehicle:get_mode()
    local armed = arming:is_armed()
    local state = round_param(param:get(STATUS_PARAM))
    local source_set = round_param(param:get(SOURCE_SET_PARAM))
    local heartbeat = round_param(param:get(HEARTBEAT_PARAM))
    local obstacle_status = round_param(param:get(OBSTACLE_STATUS_PARAM))
    local now_ms = millis()

    if last_state == nil then
        last_state = state
        last_source_set = source_set
        last_obstacle_status = obstacle_status
        last_heartbeat = heartbeat
        last_heartbeat_seen_ms = now_ms
        last_mode = mode
        last_armed = armed
        notice("Brake SLAM beeper Lua loaded")
        if obstacle_status ~= OBSTACLE_STATUS_NONE then
            report_obstacle_status(obstacle_status)
            last_obstacle_report_ms = now_ms
        end
        return update, UPDATE_MS
    end

    if heartbeat ~= last_heartbeat then
        last_heartbeat = heartbeat
        last_heartbeat_seen_ms = now_ms
        heartbeat_seen_change = true
        stale_announced = false
    end

    if state == STATE_CALIBRATION_ACTIVE and not bridge_fresh(now_ms) then
        if not stale_announced then
            warn("SLAM calibration heartbeat lost; stopping reminder beeps")
            stale_announced = true
        end
        return update, UPDATE_MS
    end

    if mode ~= last_mode then
        last_mode = mode
        notice(string.format("BEEP: FC mode changed; mode=%d", mode))
        report_mode_truth(mode, state, bridge_fresh(now_ms))
        last_mode_truth_ms = now_ms
        play("MFT220L16A") -- Single short "click" beep: Mode changed
    end

    if armed ~= last_armed then
        last_armed = armed
        if armed then
            notice("BEEP: Vehicle ARMED")
            play("MFT220L16AAA") -- Three rapid beeps: Vehicle Armed
        else
            notice("Vehicle DISARMED")
        end
    end

    if state ~= last_state or source_set ~= last_source_set then
        on_state(state, source_set)
        last_state = state
        last_source_set = source_set
    end

    if obstacle_status ~= last_obstacle_status then
        report_obstacle_status(obstacle_status)
        last_obstacle_status = obstacle_status
        last_obstacle_report_ms = now_ms
    elseif obstacle_status ~= OBSTACLE_STATUS_NONE and now_ms - last_obstacle_report_ms >= OBSTACLE_REPORT_MS then
        report_obstacle_status(obstacle_status)
        last_obstacle_report_ms = now_ms
    end

    if state == STATE_CALIBRATION_ACTIVE then
        if now_ms - last_active_beep_ms > 10000 then
            notice("BRAKE calibration still active.")
            last_active_beep_ms = now_ms
        end
    end

    if mode == MODE_POSHOLD and now_ms - last_mode_truth_ms >= POSHOLD_REPORT_MS then
        report_mode_truth(mode, state, bridge_fresh(now_ms))
        last_mode_truth_ms = now_ms
    elseif mode == MODE_BRAKE and now_ms - last_mode_truth_ms >= BRAKE_REPORT_MS then
        report_mode_truth(mode, state, bridge_fresh(now_ms))
        last_mode_truth_ms = now_ms
    end

    return update, UPDATE_MS
end

return update, 1000
