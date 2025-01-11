--------------------------------------------------------------------------------------------------
-- 
--  Autopilot for Sea King
--  ======================
--
--  The autopilot has 4 horizontal modes (TAXI, HOVR, CRUS & WNAV), and 2 vertical modes 
--  (HOLD, TRAN).
-- 
--  Horizontal modes:
--
--      TAXI: used for moving on the ground: will not take off, vertical modes disabled.
--          - WS: control ground speed.
--          - AD or LR: horizontal ground steering (heading based)
--
--      HOVR: hover over a GPS position.
--          - WSAD: move the hover position forward/back/left/right. This adjusts the target offset 
--            relative to the current position, functioning as a way to "shift" hover position.
--          - LR: adjust heading hold, always realtive to current heading.
--
--      CRUS: move forward at a constant speed & altitude
--          - WS: change cruise speed.
--          - AD or LR: adjust heading hold, relative to current heading.
--      
--      WNAV: waypoint navigation, follow waypoints automatically.
--          - WS: change cruise speed (same as CRUS).
--          - AD or LR: no effect.
--
-- Vertical modes:
-- 
--      LEVL: hold the current BARO altitude.
--          - UD: adjust the altitude setting, always relative to the current altitude.
-- 
--      ALTH: altitude hold - moves to the altitude set on the pedestal control.
--          - UD: adjusts the 
--
-- MC Inputs:
--      B1: TAXI Mode
--      B2: HOVR Mode
--      B3: CRUS Mode 
--      B4: WNAV Mode 
--      B5: LEVL Mode 
--      B6: ALTH Mode 
--      N1: WS 
--      N2: AD 
--      N3: LR 
--      N4: UD
--      N5: Alt Level Setting (ft)
--      B7: Waypoint Valid
--      N7: Waypoint X 
--      N8: Waypoint Y
--      N20-32: 6DOF Inputs
-- 
-- MC Oututs:
--      B1: AP Disconnect buzzer
--      N1: Pitch
--      N2: Roll 
--      N3: Yaw 
--      N4: Collective
--      N5: Current HMode
--      N6: Current VMode
--
--------------------------------------------------------------------------------------------------

require("Tom.Math")
require("Tom.PID")
require("Tom.RingBuffer")

TAXI = 0
HOVR = 1
CRUS = 2
WNAV = 3

LEVL = 0
ALTH = 1

THRUST_FAN_FACTOR = 2           -- how much to multiply pitch to get the thrust fan output

TAXI_COLLECTIVE = 0.2           -- collective to use for taxiing
TAXI_MAX_PITCH = 0.32           -- maximum pitch to use when taxiing     
TAXI_MIN_PITCH = -0.35          -- minimum pitch to use when taxiing
TAXI_MAX_SPEED = 5              -- maximum taxi speed (m/s)
TAXI_MIN_SPEED = -1             -- minimum taxi speed (m/s)
TAXI_HEADING_GAIN = 10          -- target heading offset at full yaw control deflection while taxiing
TAXI_MAX_YAW_RATE = 20.0        -- maximum rate of yaw (degrees per second) while taxiing

HOVR_MAX_VSPEED = 15.0          -- maximum vertical speed (m/s)
HOVR_TO_TAXI_RALT = 0.85        -- maximum radar alt that we can transition from HOVR to TAXI
HOVR_MAX_SPEED = 80.0           -- maximum forward speed (m/s)

MAX_ROLL_ANGLE = 25             -- maximum angle of roll (degrees)

NUM_INIT_FRAMES = 5             -- frames required to initialise

initFrames = 0                  -- how many init frames have been run
hMode = TAXI                    -- current horizontal mode
vMode = LEVL                    -- current vertical mode
tgtSpeed = 0                    -- the speed to hold 
tgtGPS = { x=0, y=0 }           -- the GPS coordinate to hover over
tgtHeading = 0                  -- the heading to face
tgtAltitude = 0                 -- the altitude to hold
tgtVSpeed = 0                   -- the vertical speed to target
display = { min=-10, max=10 }   -- the display constraints

-- Input: the target yaw rate, Output: the yaw control surface (tail rotor)
yawRatePID = PID:new({
    kp = 0.012, ki = 0.001, kd = 0.02, minOut = -1, maxOut = 1
})

-- Input: the angular difference, Output: the target yaw rate
headingPID = PID:new({
    kp = 2.0, ki = 0, kd = 0, minOut = -TAXI_MAX_YAW_RATE, maxOut = TAXI_MAX_YAW_RATE,
})

-- Input: the target roll angle, Output: the roll control surface
rollAnglePID = PID:new({ kp = 0.07, ki = 0.0002, kd = 1, minOut = -1, maxOut = 1, gain = 1.8 })

-- Input: the target vertical speed (m/s), Output: the collective control surface
vSpeedPID = PID:new({
    kp = 0.025, ki = 0, kd = 0, minOut = 0, maxOut = 1, bias = 0.4
})

-- Input: the target forward speed (m/s), Output: the pitch control surface.
taxiSpeedPID = PID:new({ kp = 0.3, minOut = TAXI_MIN_PITCH, maxOut = TAXI_MAX_PITCH })

-- Input: the target forward speed (m/s), Output: the pitch control surface.
fwSpeedPID = PID:new({
    kp = 0.03, ki = 0.00003, kd = 0, minOut = -1, maxOut = 1
})

-- An acceleration limiter: controls one of the output control surfaces. Used to limit acceleration 
-- in the controls and preventing an overly aggressive response.
AccelLimiter = {
    new = function(cls, p)
        local p = p or {}
        return {
            current = 0,
            target = 0,
            min = p.min or -1,
            max = p.max or 1,
            accel = p.accel or 0,
            update = cls.update
        }
    end,
    update = function(self, dt)
        local acc = self.accel * (dt or (1/60))
        local newCurrent = self.current + clamp(self.target - self.current, -acc, acc)
        self.current = clamp(newCurrent, self.min, self.max)
        return self.current
    end,
}
act = {
    yaw = AccelLimiter:new({ accel=2.0 }),
    roll = AccelLimiter:new({ accel=2.0 }),
    pitch = AccelLimiter:new({ accel=0.25, min=-0.5, max=0.75 }),
    collective = AccelLimiter:new({ accel=0.25, min=0, max=1 }),
    thrustFan = AccelLimiter:new({ accel=0.25, min=0, max=1 }),
    fwSpeed = AccelLimiter:new({ accel=3, min=-10, max=HOVR_MAX_SPEED })
}

function onTick()
    local taxiMode = input.getBool(1)
    local hovrMode = input.getBool(2)
    local crusMode = input.getBool(3)
    local wnavMode = input.getBool(4)
    local levlMode = input.getBool(5)
    local althMode = input.getBool(6)

    local wpValid = input.getBool(7)
    local wpDisconnect = hMode == WNAV and not wpValid
    if wpDisconnect then
        vMode = CRUS
    end

    local controls = {
        ws = clamp(input.getNumber(22), -1, 1), ad = clamp(input.getNumber(23), -1, 1),
        lr = clamp(input.getNumber(24), -1, 1), ud = clamp(input.getNumber(25), -1, 1),
    }
    local altSetting = input.getNumber(26)
    local wp = { x = input.getNumber(19), y = input.getNumber(20) }

    -- https://steamcommunity.com/sharedfiles/filedetails/?id=2936283512
    local worldVelocity = { x=input.getNumber(13), y=input.getNumber(14), z=input.getNumber(15) }
    local bearing = input.getNumber(6)
    local bearingVector = vFromPolar(bearing, 0)
    local rightVector = vFromPolar(bearing + math.pi / 2, 0)
    local phys = {
        pitchTilt = radToDeg(input.getNumber(5)),
        rollTilt = radToDeg(input.getNumber(4)),
        bearing = radToDeg(bearing),
        pitchAngVel = radToDeg(input.getNumber(11)),
        yawAngVel = radToDeg(input.getNumber(12)),
        rollAngVel = radToDeg(input.getNumber(10)),
        gps = { x = input.getNumber(1), y = input.getNumber(2) },
        baroAlt = input.getNumber(3), radarAlt = input.getNumber(21),
        fwSpeed = vDot(bearingVector, worldVelocity),
        rtSpeed = vDot(rightVector, worldVelocity),
        upSpeed = worldVelocity.z,
    }

    -- if we are initializing then do that
    initFrames = initFrames + 1
    if initFrames <= NUM_INIT_FRAMES then
        tgtHeading = phys.bearing
        tgtAltitude = phys.baroAlt
        return
    end

    -- yaw controls: TAXI or HOVR mode
    if hMode == TAXI or hMode == HOVR then
        -- local deflect = clamp(controls.ad + controls.lr, -1, 1) -- either AD or LS controls allowed
        local deflect = controls.lr
        if math.abs(deflect) > 0.1 then
            -- set target heading if there is some left/right control input
            tgtHeading = phys.bearing + deflect * TAXI_HEADING_GAIN
        end
    end

    -- update the debug PID
    local inputGain = input.getNumber(26)
    local debugPID = nil -- rollAnglePID -- taxiSpeedPID -- fwSpeedPID
    if debugPID then
        local divisor = math.max(input.getNumber(27), 1)
        debugPID.kp = input.getNumber(28) / divisor
        debugPID.ki = input.getNumber(29) / divisor
        debugPID.kd = input.getNumber(30) / divisor
    end
    display = { min = input.getNumber(31), max = input.getNumber(32) }

    if hMode == TAXI then
        act.collective.target = TAXI_COLLECTIVE
        local tgtFwSpeed = clamp(controls.ws * TAXI_MAX_SPEED, TAXI_MIN_SPEED, TAXI_MAX_SPEED)
        local pitch = taxiSpeedPID:update(tgtFwSpeed, phys.fwSpeed)
        act.pitch.target = pitch
        act.thrustFan.target = pitch * THRUST_FAN_FACTOR
        --pitch = clamp(controls.ws * TAXI_MAX_PITCH, TAXI_MIN_PITCH, TAXI_MAX_PITCH)

        -- check whether to switch to HOVR mode
        if hovrMode then
            hMode = HOVR
            tgtAltitude = phys.baroAlt
        end
    elseif hMode == HOVR then
        -- check whether to switch to TAXI mode
        if taxiMode then
            if phys.radarAlt <= HOVR_TO_TAXI_RALT then -- must be below max height
                hMode = TAXI
            else
                wpDisconnect = true -- warn that we are too high 
            end
        end
        local tgtVSpeed = clamp(controls.ud * HOVR_MAX_VSPEED, -HOVR_MAX_VSPEED, HOVR_MAX_VSPEED)

        local fracUp = 1 -- math.cos(math.abs(phys.pitchTilt))
        act.collective.target = vSpeedPID:update(tgtVSpeed, phys.upSpeed) / fracUp

        act.fwSpeed.target = clamp(controls.ws * HOVR_MAX_SPEED, -HOVR_MAX_SPEED, HOVR_MAX_SPEED)
        local tgtFwSpeed = act.fwSpeed:update()
        local fwSpeed = fwSpeedPID:update(tgtFwSpeed, phys.fwSpeed)
        act.pitch.target = fwSpeed
        act.thrustFan.target = fwSpeed * THRUST_FAN_FACTOR

        local tgtRollAngle = controls.ad * MAX_ROLL_ANGLE
        act.roll.target = rollAnglePID:update(tgtRollAngle, phys.rollTilt)
    elseif hMode == CRUS then
    else -- hMode == WNAV
    end

    -- yaw/heading autopilot
    local yawDiff = angularDiffDeg(phys.bearing, tgtHeading)
    local tgtYawRate = headingPID:update(yawDiff, 0)
    act.yaw.target = yawRatePID:update(tgtYawRate, phys.yawAngVel)

    -- Write outputs
    output.setBool(1, wpDisconnect)
    output.setNumber(1, act.pitch:update())
    output.setNumber(2, act.roll:update())
    output.setNumber(3, act.yaw:update())
    output.setNumber(4, act.collective:update())
    output.setNumber(5, hMode)
    output.setNumber(6, vMode)
    output.setNumber(7, clutchFactor(act.thrustFan:update()))

    -- add output for the debug PID (if enabled)
    if debugPID and debugPID.last then
        buffer:push(debugPID.last)
    end
end

buffer = RingBuffer:new(96)

function clutchFactor(x) return lerpClamp(x, 0, 0.3, 1.0, 1.0) end

function onDraw()
    local w, h = screen.getWidth(), screen.getHeight()
    local inf = 1/0
    local tgtCur = display
    local out = {min=-1,max=1} -- { min=inf, max=-inf }
    if tgtCur.min == tgtCur.max then
        tgtCur.min = tgtCur.min - 0.5
        tgtCur.max = tgtCur.max + 0.5
    end
    if out.min == out.max then
        out.min = out.min - 0.5
        out.max = out.max + 0.5
    end
    local x = 0
    local last = nil
    screen.setColor(64, 64, 64)
    local zeroY = lerp(0, tgtCur.min, h, tgtCur.max, 0)
    screen.drawLine(0, zeroY, w, zeroY)
    for v in buffer:iter() do
        local t = lerp(v.target, tgtCur.min, h, tgtCur.max, 0)
        local c = lerp(v.current, tgtCur.min, h, tgtCur.max, 0)
        local o = lerp(v.output, out.min, h, out.max, 0)
        if last then
            screen.setColor(255, 0, 0)
            screen.drawLine(last.x, last.t, x, t)
            screen.setColor(255, 255, 0)
            screen.drawLine(last.x, last.c, x, c)
            --screen.setColor(0, 0, 255)
            --screen.drawLine(last.x, last.o, x, o)
        end
        last = { x=x, t=t, c=c, o=o }
        x = x + 1
    end
end