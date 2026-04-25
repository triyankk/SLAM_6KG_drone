-- Install on the flight controller SD card as:
--   APM/scripts/jetson_nogps_status.lua
-- Uses SCR_USER1 for the Jetson SLAM bridge state code and SCR_USER2 for the target EKF source set.

local STATUS_PARAM = "SCR_USER1"
local SOURCE_SET_PARAM = "SCR_USER2"
local UPDATE_MS = 1000

local STATE_NONE = 0
local STATE_JETSON_BOOT = 10
local STATE_SENSOR_CHECK_PASSED = 12
local STATE_FLOW_STARTED = 40
local STATE_SOURCE_SET_ACTIVE = 50
local STATE_POSHOLD_READY = 54
local STATE_CALIBRATION_ACTIVE = 70
local STATE_CALIBRATION_COMPLETE_RTL = 71
local STATE_SLAM_FLIGHT_ACTIVE = 72
local STATE_SOURCE_SWITCH_FAILED = 82
local STATE_SOURCE_SWITCH_NO_ACK = 83

local last_state = nil
local last_source_set = nil
local last_home_is_set = nil

local function round_param(value)
    if value == nil then
        return 0
    end
    return math.floor(value + 0.5)
end

local function send_notice(text)
    gcs:send_text(5, text)
end

local function send_warning(text)
    gcs:send_text(4, text)
end

local function relay_state_change(state_code, source_set_id)
    if state_code == STATE_JETSON_BOOT then
        send_notice("Jetson SLAM bridge initiated")
        return
    end

    if state_code == STATE_SENSOR_CHECK_PASSED then
        send_notice("SLAM sensor quick check passed")
        return
    end

    if state_code == STATE_FLOW_STARTED then
        send_notice("SLAM odometry stream started")
        return
    end

    if state_code == STATE_SOURCE_SET_ACTIVE then
        if source_set_id > 0 then
            send_notice(string.format("SLAM/ExternalNav ACTIVE: EKF source_set=%d", source_set_id))
        else
            send_notice("SLAM/ExternalNav source active")
        end
        return
    end

    if state_code == STATE_POSHOLD_READY then
        if source_set_id > 0 then
            send_notice(string.format("SLAM ready for PosHold on source set %d", source_set_id))
        else
            send_notice("SLAM ready for PosHold")
        end
        return
    end

    if state_code == STATE_CALIBRATION_ACTIVE then
        send_notice("SLAM calibration active")
        return
    end

    if state_code == STATE_CALIBRATION_COMPLETE_RTL then
        send_notice("Calibration complete, switching to RTL")
        return
    end

    if state_code == STATE_SLAM_FLIGHT_ACTIVE then
        send_notice("No-GPS SLAM flight active")
        return
    end

    if state_code == STATE_SOURCE_SWITCH_FAILED then
        send_warning("EKF source switch failed")
        return
    end

    if state_code == STATE_SOURCE_SWITCH_NO_ACK then
        send_warning("EKF source switch no ack")
        return
    end
end

local function update()
    local current_state = round_param(param:get(STATUS_PARAM))
    local current_source_set = round_param(param:get(SOURCE_SET_PARAM))
    local home_is_set = ahrs:home_is_set()

    if last_state == nil then
        last_state = current_state
        last_source_set = current_source_set
        last_home_is_set = home_is_set
        send_notice("Jetson SLAM relay Lua loaded")
        return update, UPDATE_MS
    end

    if current_state ~= last_state then
        relay_state_change(current_state, current_source_set)
        last_state = current_state
        last_source_set = current_source_set
    elseif current_source_set ~= last_source_set then
        last_source_set = current_source_set
    end

    if last_home_is_set ~= nil and home_is_set ~= last_home_is_set then
        if home_is_set then
            send_notice("FC home position is set")
        else
            send_warning("FC home position cleared")
        end
        last_home_is_set = home_is_set
    end

    return update, UPDATE_MS
end

return update, 2000
