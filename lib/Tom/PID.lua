---@section PID
PID = {
    new = function(cls, p)
        local p = p or {}
        return {
            -- The proportional P term for the PID
            kp = p.kp or 0,
            -- The integral I term for the PID
            ki = p.ki or 0,
            -- The derivative D term for the PID
            kd = p.kd or 0,
            -- The bias is the base output that is always added before the PID runs
            bias = p.bias or 0,
            -- The minimum output value (clamp)
            minOut = p.minOut or -(1/0),
            -- The maximum output value (clamp)
            maxOut = p.maxOut or (1/0),
            -- Additional set point gain 
            gain = p.gain or 1.0,
            last = nil,
            errorPrior = 0,
            integralPrior = 0,
            update = cls.update,
        }
    end,
    update = function(self, target, current)
        local target = target * self.gain
        local error = target - current
        local integral = self.integralPrior + error
        local derivative = error - self.errorPrior
        local valueOut = self.kp*error + self.ki*integral + self.kd*derivative + self.bias
        self.errorPrior = error
        local output = math.min(math.max(valueOut, self.minOut), self.maxOut)

        -- clamping anti-windup: only accumulate the error if we are not saturating the output
        -- or the error is making it better rather than worse.
        -- See: https://youtu.be/NVLXCwc8HzM?t=509
        if output == valueOut or error * valueOut <= 0 then
            self.integralPrior = integral
        end
        self.last = { target=target, current=current, output=output }
        return output
    end,
}
---@endsection
