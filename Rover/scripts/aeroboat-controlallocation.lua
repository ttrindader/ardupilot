-- aeroboat-controlallocation.lua  (sandbox-safe + sem tonumber direto em param)

-- ===== Mini util =====
local funcs = {}
function funcs.toDegrees(rad) return (rad or 0) * 180.0 / math.pi end
function funcs.mapTo360(deg) local v = (deg or 0) % 360; if v < 0 then v = v + 360 end; return v end

-- Converte valor para número só se for number ou string; senão usa fallback
local function to_num(x, fallback)
  if type(x) == "number" then return x end
  if type(x) == "string" then
    local n = tonumber(x); if n ~= nil then return n end
  end
  return fallback
end

-- Lê param com fallback, sem chamar tonumber em tipos não numéricos
local function pget(name, fallback)
  local v = nil
  if param and param.get then v = param:get(name) end
  return to_num(v, fallback)
end

-- ===== Constantes =====
local CONTROL_OUTPUT_THROTTLE = 3
local CONTROL_OUTPUT_YAW      = 4
local LOW_TRIM, MID_TRIM      = 1100, 1500
local RADIO_CHANNEL_BANDWIDTH = 450
local desired_yaw             = -1.0

local MAV_SEVERITY  = { EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7 }
local DRIVING_MODES = { MANUAL=0, STEERING=3, HOLD=4, AUTO=10, GUIDED=15 }

local previous_driving_mode = (vehicle and vehicle.get_mode and vehicle:get_mode()) or DRIVING_MODES.MANUAL

-- Gate simples via param (sem usar Parameter())
local function system_started()
  local v = pget('BATT_SOC_SYSSTART', nil)
  if v == nil then v = pget('SCR_USER1', 0) end
  return v ~= 0
end

-- ===== Alocação de controle =====
local function new_control_allocation(t, s)
  local eps, aloc = 1e-4, 400
  t, s = (t or 0), (s or 0)

  local hip = math.sqrt(t*t + s*s) + eps
  local nTa, nSa = aloc * t / hip, aloc * s / hip
  local denom = math.abs(nTa) + math.abs(nSa) + eps
  local T, S = math.abs(nTa) / denom, math.abs(nSa) / denom

  local nft, nfs = t * T * aloc, s * S * aloc
  local naloc_right = math.floor(nft + nfs)
  local naloc_left  = math.floor(nft - nfs)

  local pwm0 = pget('SERVO1_TRIM', LOW_TRIM)
  local pwm1 = pget('SERVO2_TRIM', LOW_TRIM)
  local pwm2 = pget('SERVO3_TRIM', LOW_TRIM)
  local pwm3 = pget('SERVO4_TRIM', LOW_TRIM)
  local pwm4 = pget('SERVO5_TRIM', LOW_TRIM)
  local pwm5 = pget('SERVO6_TRIM', LOW_TRIM)

  local function set(ch, pwm) SRV_Channels:set_output_pwm_chan_timeout(ch, pwm, 300) end
  local function clamp_pwm(x)
    if x == nil then return MID_TRIM end
    if x > 1700 then return 1700 elseif x < 1000 then return 1000 else return x end
  end

  -- Right: ch1,ch2 fwd; ch4 rev
  if naloc_right >= 0 then
    set(1, clamp_pwm(2*naloc_right + pwm1)); set(2, clamp_pwm(2*naloc_right + pwm2)); set(4, pwm4)
  else
    set(1, pwm1); set(2, pwm2); set(4, clamp_pwm(pwm4 - 2*naloc_right))
  end

  -- Left: ch0,ch3 fwd; ch5 rev
  if naloc_left >= 0 then
    set(0, clamp_pwm(2*naloc_left + pwm0)); set(3, clamp_pwm(2*naloc_left + pwm3)); set(5, pwm5)
  else
    set(0, pwm0); set(3, pwm3); set(5, clamp_pwm(pwm5 - 2*naloc_left))
  end
end

-- ===== Loop principal =====
local function update()
  local vehicle_type = pget('SCR_USER5', 1)
  if vehicle_type ~= 1 then
    gcs:send_text(MAV_SEVERITY.INFO, "Not a boat; script idle.")
    return update, 1000
  end

  if not system_started() then
    gcs:send_text(MAV_SEVERITY.WARNING, "BATT_SOC_SYSSTART=0; waiting...")
    return update, 1000
  end

  local mode = (vehicle and vehicle.get_mode and vehicle:get_mode()) or DRIVING_MODES.MANUAL
  if mode == DRIVING_MODES.STEERING then
    if vehicle and vehicle.set_mode then vehicle:set_mode(previous_driving_mode) end
    mode = previous_driving_mode
    gcs:send_text(MAV_SEVERITY.WARNING, "STEERING not allowed; restoring previous mode")
  elseif mode ~= previous_driving_mode then
    previous_driving_mode = mode
  end

  if not (arming and arming.is_armed and arming:is_armed()) then
    desired_yaw = -1.0
    local trims = {
      pget('SERVO1_TRIM', LOW_TRIM), pget('SERVO2_TRIM', LOW_TRIM),
      pget('SERVO3_TRIM', LOW_TRIM), pget('SERVO4_TRIM', LOW_TRIM),
      pget('SERVO5_TRIM', LOW_TRIM), pget('SERVO6_TRIM', LOW_TRIM),
    }
    for ch=0,5 do SRV_Channels:set_output_pwm_chan_timeout(ch, trims[ch+1], 3000) end
    gcs:send_text(MAV_SEVERITY.INFO, "Boat disarmed; holding trims.")
    return update, 2000
  end

  local steering, throttle = 0, 0
  if mode == DRIVING_MODES.MANUAL then
    local TRIM3 = pget('RC3_TRIM', MID_TRIM)
    local TRIM1 = pget('RC1_TRIM', MID_TRIM)
    local rc3   = to_num(rc and rc.get_pwm and rc:get_pwm(3), TRIM3)
    local rc1   = to_num(rc and rc.get_pwm and rc:get_pwm(1), TRIM1)
    throttle    = (TRIM3 - rc3) / RADIO_CHANNEL_BANDWIDTH
    steering    = (rc1 - TRIM1) / RADIO_CHANNEL_BANDWIDTH
    new_control_allocation(throttle, steering)
    return update, 200

  elseif mode == DRIVING_MODES.AUTO or mode == DRIVING_MODES.GUIDED then
    if desired_yaw == -1.0 then
      desired_yaw = funcs.mapTo360(funcs.toDegrees(ahrs and ahrs.get_yaw and ahrs:get_yaw()))
    end
    local TRIM3 = pget('RC3_TRIM', MID_TRIM)
    local TRIM1 = pget('RC1_TRIM', MID_TRIM)
    local rc3   = to_num(rc and rc.get_pwm and rc:get_pwm(3), TRIM3)
    local rc1   = to_num(rc and rc.get_pwm and rc:get_pwm(1), TRIM1)
    throttle    = (TRIM3 - rc3) / RADIO_CHANNEL_BANDWIDTH
    steering    = (rc1 - TRIM1) / RADIO_CHANNEL_BANDWIDTH

    if math.abs(steering) > 0.10 or math.abs(throttle) > 0.10 then
      desired_yaw = funcs.mapTo360(funcs.toDegrees(ahrs and ahrs.get_yaw and ahrs:get_yaw()))
      new_control_allocation(throttle, steering)
    else
      steering = to_num(vehicle and vehicle.get_control_output and vehicle:get_control_output(CONTROL_OUTPUT_YAW), 0)
      throttle = to_num(vehicle and vehicle.get_control_output and vehicle:get_control_output(CONTROL_OUTPUT_THROTTLE), 0)
      new_control_allocation(throttle, steering)
    end
    return update, 200
  else
    steering = to_num(vehicle and vehicle.get_control_output and vehicle:get_control_output(CONTROL_OUTPUT_YAW), 0)
    throttle = to_num(vehicle and vehicle.get_control_output and vehicle:get_control_output(CONTROL_OUTPUT_THROTTLE), 0)
    new_control_allocation(throttle, steering)
    return update, 200
  end
end

return update, 100

