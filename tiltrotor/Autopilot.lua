require("Tom.Math")
require("Tom.PID")
require("Tom.RingBuffer")

RALT_ON_GROUND = 3.0            -- radar altimeter reading that we consider to be on the ground.

YAW_GAIN = 15.0                 -- target degrees per second at max deflection
YAW_ROTOR_FACTOR = 0 -- 0.15    -- how much to tilt rotors compared to pitch for yaw control

PITCH_RATE_GAIN = 20.0          -- target degrees per second at max deflection (1.0)
PITCH_ANGLE_GAIN = 45.0         -- target pitch angle at max deflection (1.0)

ROLL_RATE_GAIN = 30.0           -- target degrees per second at max deflection (1.0)
ROLL_ANGLE_GAIN = 45.0          -- target roll angle at max deflection (1.0)

VSPEED_GAIN = 20.0              -- target vertical speed at max deflection (1.0)

display = { min=-10, max=10, out=false }   -- the display constraints

yawRatePID = PID:new({
    kp = 0.1, ki = 0, kd = 0, minOut = -1, maxOut = 1, gain = 1.3 * YAW_GAIN
})
pitchRatePID = PID:new({
    kp = -0.01, ki = -0.0, kd = -0, minOut = -1, maxOut = 1, gain = PITCH_RATE_GAIN
})
pitchAnglePID = PID:new({
    kp = 0.2, ki = 0.002, kd = 3.0, minOut = -1, maxOut = 1, gain = PITCH_ANGLE_GAIN
})
rollRatePID = PID:new({
    kp = 0.01, ki = 0, kd = 0, minOut = -1, maxOut = 1, gain = ROLL_RATE_GAIN
})
rollAnglePID = PID:new({
    kp = 0.1, ki = 0, kd = 1.0, minOut = -1, maxOut = 1, gain = ROLL_ANGLE_GAIN
})
vSpeedPID = PID:new({
    kp = 0.15, ki = 0.00001, kd = 0.5, minOut = -1, maxOut = 1, bias = 0.4, gain = VSPEED_GAIN
})

Limiter = {
    new = function(cls, p)
        local p = p or {}
        return {
            accel = p.accel,
            min = p.min,
            max = p.max,
            current = 0,
            target = 0,
            update = cls.update,
        }
    end,
    update = function(self, dt)
        local acc = self.accel * (dt or (1/60))
        local newCurrent = self.current + clamp(self.target - self.current, -acc, acc)
        self.current = clamp(newCurrent, self.min, self.max)
        return self.current
    end,
}

function makeRotor()
    return {
        -- NOTE: lower than -0.2 and it will hit the wing.
        rotor = Limiter:new({ accel = 10, min = -0.2, max = 1 }),
        collect = Limiter:new({ accel = 10, min = 0, max = 1 }),
        pitch = Limiter:new({ accel = 10, min = -1, max = 1 }),
        roll = Limiter:new({ accel = 10, min = -1, max = 1 }),
    }
end

act = {
    left = makeRotor(),
    right = makeRotor(),
}

function onTick()
    -- Control          Helicopter              Plane
    -- ========================================================================================
    -- WS               Rotor Angle             Rotor Angle + Collective
    -- AD               Yaw                     Rudder (Yaw)
    -- UD               Collective              Elevators (Pitch)
    -- LR               Roll                    Ailerons (Roll)

    local controls = {
        ws = clamp(input.getNumber(22), -1, 1), ad = clamp(input.getNumber(23), -1, 1),
        ud = clamp(input.getNumber(24), -1, 1), lr = clamp(input.getNumber(25), -1, 1),
    }

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
    local rotorPosLeft = input.getNumber(19)
    local rotorPosRight = input.getNumber(20)

    -- update the debug PID
    local debugPID = nil -- vSpeedPID
    if debugPID then
        local divisor = math.max(input.getNumber(27), 1)
        debugPID.gain = input.getNumber(26)
        debugPID.kp = input.getNumber(28) / divisor
        debugPID.ki = input.getNumber(29) / divisor
        debugPID.kd = input.getNumber(30) / divisor
    end
    display = { min = input.getNumber(31), max = input.getNumber(32), out = input.getBool(1) }

    -- Collective: needed for testing
    act.left.collect.target = controls.ud
    act.right.collect.target = controls.ud

    -- Rotor angle
    local tgtSpeed = controls.ws
    
    -- Pitch angle & pitch rate
    local tgtPitchAngle = 0 -- helicopter mode holds a flat pitch -controls.ws
    local tgtPitchRate = pitchAnglePID:update(tgtPitchAngle, phys.pitchTilt)
    local pitchControl = pitchRatePID:update(tgtPitchRate, phys.pitchAngVel)

    -- Yaw rate
    local tgtYawRate = controls.lr
    local yawControl = yawRatePID:update(tgtYawRate, phys.yawAngVel)

    -- Roll rate
    local tgtRollAngle = controls.ad
    local tgtRollRate = rollAnglePID:update(tgtRollAngle, phys.rollTilt)
    local rollControl = rollRatePID:update(tgtRollRate, phys.rollAngVel)

    -- Collective & vertical speed
    local tgtVSpeed = controls.ud
    local collectControl = vSpeedPID:update(tgtVSpeed, phys.upSpeed)

    -- Control the left and right rotor angle & pitch
    act.left.rotor.target = controls.ws + (yawControl * YAW_ROTOR_FACTOR)
    act.left.pitch.target = pitchControl + yawControl
    act.left.roll.target = -rollControl
    act.left.collect.target = collectControl
    act.right.rotor.target = controls.ws + (yawControl * YAW_ROTOR_FACTOR)
    act.right.pitch.target = pitchControl - yawControl
    act.right.roll.target = rollControl
    act.right.collect.target = collectControl

    -- Limit rotor tilt when too close to the ground
    local rotorLimit = 0.5 -- phys.radarAlt < RALT_ON_GROUND and 0.5 or 1.0
    act.left.rotor.max = rotorLimit
    act.right.rotor.max = rotorLimit

    -- Write outputs
    output.setNumber(1, act.left.rotor:update())
    output.setNumber(2, act.left.collect:update())
    output.setNumber(3, act.left.pitch:update())
    output.setNumber(4, act.left.roll:update())
    output.setNumber(5, act.right.rotor:update())
    output.setNumber(6, act.right.collect:update())
    output.setNumber(7, act.right.pitch:update())
    output.setNumber(8, act.right.roll:update())

    -- add output for the debug PID (if enabled)
    if debugPID and debugPID.last then
        buffer:push(debugPID.last)
    end
end

buffer = RingBuffer:new(96)

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
            if display.out then
                screen.setColor(0, 0, 255)
                screen.drawLine(last.x, last.o, x, o)
            end
        end
        last = { x=x, t=t, c=c, o=o }
        x = x + 1
    end
end