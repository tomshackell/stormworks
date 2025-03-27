require("Tom.Math")
require("Tom.PID")
require("Tom.RingBuffer")

RALT_ON_GROUND = 3.0            -- radar altimeter reading that we consider to be on the ground.


display = { min=-10, max=10, out=false }   -- the display constraints

-- Used to limit something: both in min/max and also in acceleration (how fast it can change)
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
        local curr = nanGuard(self.current)
        local newCurrent = curr + clamp(nanGuard(self.target) - curr, -acc, acc)
        self.current = clamp(newCurrent, self.min, self.max)
        return self.current
    end,
}

function makeRotor()
    return {
        -- NOTE: lower than -0.2 and it will hit the wing.
        rotor = Limiter:new({ accel = 10, min = -0.2, max = 1 }),
        collect = Limiter:new({ accel = 10, min = -1, max = 1 }),
        pitch = Limiter:new({ accel = 10, min = -1, max = 1 }),
        roll = Limiter:new({ accel = 10, min = -1, max = 1 }),
    }
end

act = {
    left = makeRotor(),
    right = makeRotor(),
    elevators = Limiter:new({ accel = 1, min = -1, max = 1 }),
    ailerons = Limiter:new({ accel = 1, min = -1, max = 1 }),
    rudders = Limiter:new({ accel = 1, min = -1, max = 1 }),
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
    local debugEnabled = input.getBool(2)
    local debugPID = AirplaneAP.yawRatePID
    if debugPID and debugEnabled then
        local divisor = math.max(input.getNumber(27), 1)
        debugPID.gain = input.getNumber(26)
        debugPID.kp = input.getNumber(28) / divisor
        debugPID.ki = input.getNumber(29) / divisor
        debugPID.kd = input.getNumber(30) / divisor
    end
    display = { min = input.getNumber(31), max = input.getNumber(32), out = input.getBool(1) }

    AirplaneAP:update(phys, controls, act)

    -- Write outputs
    output.setNumber(1, act.left.rotor:update())
    output.setNumber(2, act.left.collect:update())
    output.setNumber(3, act.left.pitch:update())
    output.setNumber(4, act.left.roll:update())
    output.setNumber(5, act.right.rotor:update())
    output.setNumber(6, act.right.collect:update())
    output.setNumber(7, act.right.pitch:update())
    output.setNumber(8, act.right.roll:update())
    output.setNumber(9, act.elevators:update())
    output.setNumber(10, act.ailerons:update())
    output.setNumber(11, act.rudders:update())

    -- add output for the debug PID (if enabled)
    if debugPID and debugPID.last then
        buffer:push(debugPID.last)
    end
end

AirplaneAP = {
    YAW_COLLECT_FACTOR = 0.4, -- how much to use collective to yaw, relative to rudders

    yawRatePID = PID:new({
        kp = 0.002, ki = 0, kd = 0.005, minOut = -1, maxOut = 1, gain = 130
    }),
    yawControlLimiter = Limiter:new({ accel = 0.1, min = -0.4, max = 0.4 }),
    simRollLimiter = Limiter:new({ accel = 0.05, min = -0.2, max = 0.2 }),
    rollRatePID = PID:new({
        kp = 0.002, ki = 0.00005, kd = 0.003, minOut = -1, maxOut = 1, gain = 30,
    }),
    rollAnglePID = PID:new({
        kp = 0.08, ki = 0, kd = 1, minOut = -1, maxOut = 1, gain = 45,
    }),
    pitchRatePID = PID:new({
        kp = 0.0001, ki = 0.0001, kd = 0.0005, minOut = -1, maxOut = 1, gain = 20,
    }),
    pitchAnglePID = PID:new({
        kp = 0.04, ki = 0, kd = 4, minOut = -1, maxOut = 1, gain = 45
    }),
    update = function(self, phys, controls, act)

        local tgtPitch = controls.ud
        local tgtPitchRate = self.pitchAnglePID:update(tgtPitch, phys.pitchTilt)
        local pitchControl = self.pitchRatePID:update(tgtPitchRate, phys.pitchAngVel)

        local tgtYawRate = controls.ad
        self.yawControlLimiter.target = self.yawRatePID:update(tgtYawRate, phys.yawAngVel)
        local yawControl = self.yawControlLimiter:update()

        self.simRollLimiter.target =
            controls.ad * lerpClamp(phys.fwSpeed, 50, 0, 75, self.simRollLimiter.max)

        local tgtRoll = controls.lr -- + self.simRollLimiter:update()
        local tgtRollRate = self.rollAnglePID:update(tgtRoll, phys.rollTilt)
        local rollControl = self.rollRatePID:update(tgtRollRate, phys.rollAngVel)

        act.left.rotor.target = 1
        act.left.collect.target = controls.ws + yawControl * self.YAW_COLLECT_FACTOR
        act.left.pitch.target = -pitchControl
        act.right.rotor.target = 1
        act.right.collect.target = controls.ws - yawControl * self.YAW_COLLECT_FACTOR
        act.right.pitch.target = -pitchControl

        act.elevators.target = pitchControl -- controls.ud
        act.ailerons.target = rollControl
        act.rudders.target = yawControl
    end,
}

HeliAP = {
    yawRatePID = PID:new({
        kp = 0.1, ki = 0, kd = 0, minOut = -1, maxOut = 1, gain = 1.3 * 15
    }),
    pitchRatePID = PID:new({
        kp = -0.01, ki = -0.0, kd = -0, minOut = -1, maxOut = 1, gain = 20
    }),
    pitchAnglePID = PID:new({
        kp = 0.2, ki = 0.002, kd = 3.0, minOut = -1, maxOut = 1, gain = 45
    }),
    rollRatePID = PID:new({
        kp = 0.01, ki = 0, kd = 0, minOut = -1, maxOut = 1, gain = 30
    }),
    rollAnglePID = PID:new({
        kp = 0.1, ki = 0, kd = 1.0, minOut = -1, maxOut = 1, gain = 45
    }),
    vSpeedPID = PID:new({
        kp = 0.15, ki = 0.00001, kd = 0.5, minOut = -1, maxOut = 1, bias = 0.4, gain = 20
    }),
    fwSpeedPID = PID:new({
        kp = 0.05, ki = 0.000001, kd = 0.4, minOut = -1, maxOut = 1, gain = 400,
    }),
    fwSpeedLimiter = Limiter:new({ accel = 3, min = -10, max = 400 }),

    update = function(self, phys, controls, act)
        -- Conversion
        local airplane = lerpClamp(phys.fwSpeed, 40, 0, 60, 1)
        local blend = function(h, a) return h * (1 - airplane) + a * airplane end

        -- Forward speed
        self.fwSpeedLimiter.target = controls.ws * self.fwSpeedLimiter.max
        local tgtSpeedNow = controls.ws -- fwSpeedLimiter:update()
        local fwSpeedControl = self.fwSpeedPID:update(tgtSpeedNow, phys.fwSpeed)
            
        -- Pitch angle & pitch rate
        local tgtPitchAngle = 0 -- helicopter mode holds a flat pitch -controls.ws
        local tgtPitchRate = self.pitchAnglePID:update(tgtPitchAngle, phys.pitchTilt)
        local pitchControl = self.pitchRatePID:update(tgtPitchRate, phys.pitchAngVel)

        -- Yaw rate
        local tgtYawRate = controls.ad
        local yawControl = self.yawRatePID:update(tgtYawRate, phys.yawAngVel)

        -- Roll rate
        local tgtRollAngle = controls.lr
        local tgtRollRate = self.rollAnglePID:update(tgtRollAngle, phys.rollTilt)
        local rollControl = self.rollRatePID:update(tgtRollRate, phys.rollAngVel)

        -- Collective & vertical speed
        local tgtVSpeed = controls.ud
        local vspeedControl = vSpeedPID:update(tgtVSpeed, phys.upSpeed)
        
        -- Control the left and right rotor angle & pitch
        local leftRotor = function(up, dn) return lerpClamp(rotorPosLeft, 0, up, 1, dn) end
        act.left.rotor.target = blend(fwSpeedControl, 1) --+ (yawControl * YAW_ROTOR_FACTOR)
        act.left.pitch.target = blend(pitchControl + yawControl, 0)
        act.left.roll.target = blend(-rollControl, 0)
        act.left.collect.target = blend(vspeedControl, fwSpeedControl)

        local rightRotor = function(up, dn) return lerpClamp(rotorPosRight, 0, up, 1, dn) end
        act.right.rotor.target = blend(fwSpeedControl, 1) -- + (yawControl * YAW_ROTOR_FACTOR)
        act.right.pitch.target = blend(pitchControl - yawControl, 0)
        act.right.roll.target = blend(rollControl, 0)
        act.right.collect.target = blend(vspeedControl, fwSpeedControl)

        -- Limit rotor tilt when too close to the ground    
        local rotorLimit = blend(phys.radarAlt, 1.0)
        act.left.rotor.max = rotorLimit
        act.right.rotor.max = rotorLimit

        act.elevators.target = controls.ud
        act.ailerons.target = controls.lr
        act.rudders.target = controls.ad
    end,
}

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