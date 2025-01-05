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
            -- How much decay the accumulated integral error per second: 0 = no decay, 1 = fully
            iDecay = p.iDecay or 0,
            -- The minimum output value (clamp)
            minOut = p.minOut or -(1/0),
            -- The maximum output value (clamp)
            maxOut = p.maxOut or (1/0),
            errorPrior = 0,
            integralPrior = 0,
            update = cls.update,
        }
    end,
    update = function(self, target, current, dt)
        local dt = dt or (1 / 60)
        local error = target - current
        local integral = (self.integralPrior + error) * (1 - (self.iDecay / dt))
        local derivative = error - self.errorPrior
        local valueOut = self.kp*error + self.ki*integral + self.kd*derivative + self.bias
        self.errorPrior = error
        self.integralPrior = integral
        return math.min(math.max(valueOut, self.minOut), self.maxOut)
    end,
}
---@endsection
