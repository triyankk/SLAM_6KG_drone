-- Optional ArduPilot Lua backup beeper for the Jetson SLAM calibration monitor.
-- Install on the flight controller SD card as:
--   APM/scripts/brake_slam_beeper.lua
--
-- Jetson remains the calibration brain. This script only relays SCR_USER1
-- state changes into GCS text and buzzer tunes if notify:play_tune is present.

local STATUS_PARAM = "SCR_USER1"
local SOURCE_SET_PARAM = "SCR_USER2"
local HEARTBEAT_PARAM = "SCR_USER3"
local UPDATE_MS = 500
local STALE_MS = 15000

local STATE_IDLE = 0
local STATE_JETSON_BOOT = 10
local STATE_SENSOR_CHECK_PASSED = 12
local STATE_SLAM_STARTED = 40
local STATE_SOURCE_SET_ACTIVE = 50
local STATE_POSHOLD_READY = 54
local STATE_CALIBRATION_WAITING_ARM = 68
local STATE_CALIBRATION_ACTIVE = 70
local STATE_CALIBRATION_COMPLETE_RTL = 71
local STATE_SLAM_FLIGHT_ACTIVE = 72
local STATE_SOURCE_SWITCH_FAILED = 82
local STATE_SOURCE_SWITCH_NO_ACK = 83

local last_state = nil
local last_source_set = nil
local last_mode = nil
local last_armed = nil
local last_heartbeat = nil
local last_heartbeat_seen_ms = 0
local heartbeat_seen_change = false
local stale_announced = false
local last_active_beep_ms = 0

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

local function on_state(state, source_set)
    if state == STATE_IDLE then
        notice("SLAM bridge idle")
    elseif state == STATE_JETSON_BOOT then
        notice("Jetson SLAM bridge initiated")
        play("MFT200L8AAA") -- Three short medium-pitch beeps: Booting
    elseif state == STATE_SENSOR_CHECK_PASSED then
        notice("SLAM sensor quick check passed")
        play("MFT240L8A") -- Single medium beep: Sensors OK
    elseif state == STATE_SLAM_STARTED then
        notice("SLAM odometry stream started")
    elseif state == STATE_SOURCE_SET_ACTIVE then
        notice(string.format("SLAM ExternalNav source active: %d", source_set))
    elseif state == STATE_POSHOLD_READY then
        notice("SLAM ready for PosHold")
        play("MFT200L16CDEF") -- Fast ascending scale: System ready for flight
    elseif state == STATE_CALIBRATION_WAITING_ARM then
        notice("Brake mode detected. Waiting for arm to start SLAM calibration.")
    elseif state == STATE_CALIBRATION_ACTIVE then
        notice("SLAM calibration active")
        play("MFT180L16GABG") -- Distinctive four-note melody: Calibration in progress
    elseif state == STATE_CALIBRATION_COMPLETE_RTL then
        notice("Calibration successful: SLAM PosHold calibration complete. Initiating RTL.")
        play("MFT160L4CDEF") -- Long, bright ascending scale: Calibration success
    elseif state == STATE_SLAM_FLIGHT_ACTIVE then
        notice("SLAM flight active")
        play("MFT240L8A") -- Single beep: Navigation active
    elseif state == STATE_SOURCE_SWITCH_FAILED then
        warn("EKF source switch failed")
        play("MFT160L8CBA") -- Descending scale: Error/Failure
    elseif state == STATE_SOURCE_SWITCH_NO_ACK then
        warn("EKF source switch no ack")
        play("MFT160L8CBA") -- Descending scale: Error/Failure
    end
end

local function update()
    local mode = vehicle:get_mode()
    local armed = arming:is_armed()
    local state = round_param(param:get(STATUS_PARAM))
    local source_set = round_param(param:get(SOURCE_SET_PARAM))
    local heartbeat = round_param(param:get(HEARTBEAT_PARAM))
    local now_ms = millis()

    if last_state == nil then
        last_state = state
        last_source_set = source_set
        last_heartbeat = heartbeat
        last_heartbeat_seen_ms = now_ms
        last_mode = mode
        last_armed = armed
        notice("Brake SLAM beeper Lua loaded")
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
        notice(string.format("Flight mode changed: mode=%d", mode))
        play("MFT220L16A") -- Single short "click" beep: Mode changed
    end

    if armed ~= last_armed then
        last_armed = armed
        if armed then
            notice("Vehicle ARMED")
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

    if state == STATE_CALIBRATION_ACTIVE then
        if now_ms - last_active_beep_ms > 10000 then
            notice("SLAM calibration active")
            play("MFT240L8A") -- Single reminder beep every 10 seconds during calibration
            last_active_beep_ms = now_ms
        end
    end

    return update, UPDATE_MS
end

return update, 1000
