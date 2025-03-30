---@section SplineInterpolator
-- Solve a linear system with a tridiagonal n*n matrix using Gaussian elimination without pivoting
local function solveTridiag(sub, diag, sup, b, n)
    for i = 2, n do
        sub[i] = sub[i] / diag[i - 1]
        diag[i] = diag[i] - sub[i] * sup[i - 1]
        b[i] = b[i] - sub[i] * b[i - 1]
    end
    b[n] = b[n] / diag[n]
    for i = n - 1, 1, -1 do
        b[i] = (b[i] - sup[i] * b[i + 1]) / diag[i]
    end
end

-- Get a value based on linearly interpolating between the values for `lidx` and `hidx`
local function getLinearInterpolatedValue(self, key, lidx, hidx)
    local lov, hiv = self.values[lidx + 1], self.values[hidx + 1]
    local lok, hik = self.keys[lidx + 1], self.keys[hidx + 1]
    local m = (hiv - lov) / (hik - lok)
    local c = hiv - m * hik
    return m * key + c
end

-- return a 'vector' of n zeros: zero-indexed, unlike normal lua 1-indexed
local function zeroVec(n)
    local r = {}
    for i = 0, n - 1 do r[i] = 0 end
    return r
end

SplineInterpolator = {
    -- Create a new spline interpolator for the given set of keys & values.
    -- Keys & values should be two arrays of the same length sorted by increasing key.
    new = function(cls, keys, values)
        assert(#values == #keys)
        local inf = 1/0
        local n = #keys
        local r = {
            keys = keys,
            values = values,
            a = zeroVec(n),
            h = zeroVec(n),
            minKey = inf,
            maxKey = -inf,
            getValue = cls.getValue,
        }
        for i = 0, n - 1 do
            r.minKey = math.min(r.minKey, keys[i + 1])
            r.maxKey = math.max(r.maxKey, keys[i + 1])
            r.h[i] = i == 0 and 0 or keys[i + 1] - keys[i]
        end
        if n > 2 then
            local sup, diag, sub = zeroVec(n-1), zeroVec(n-1), zeroVec(n-1)
            for i = 1, n - 2 do
                diag[i] = (r.h[i] + r.h[i + 1]) / 3
                sup[i] = r.h[i + 1] / 6
                sub[i] = r.h[i] / 6
                r.a[i] = (values[i + 2] - values[i + 1]) / r.h[i + 1] -
                         (values[i + 1] - values[i]) / r.h[i]
            end
            solveTridiag(sub, diag, sup, r.a, n - 2)
        end
        return r
    end,

    -- Get the value for a particular key by interpolating between the known values
    getValue = function(self, key, extendNonLinear)
        local n = #self.keys
        -- NOTE: if we wanted this smoother we could blend the end values into the linear
        -- interpolation, currently it can create a "kink" where it transitions.
        if not extendNonLinear then
            if key <= self.minKey then
                return getLinearInterpolatedValue(self, key, 0, 1)
            elseif key >= self.maxKey then
                return getLinearInterpolatedValue(self, key, n - 2, n - 1)
            end
        end
        local gap, previous = 1, -(1/0)
        -- NOTE: this extends the function beyond the boundaries if 'extendNonLinear' is true.
        -- It works, and gives smooth curves .. but those curves can do weird things.
        if key <= self.minKey then
            previous, gap = self.keys[1], 1
        elseif key >= self.maxKey then
            previous, gap = self.keys[n - 1], n - 1
        else
            -- At the end of this iteration, "gap" will contain the index of the interval
            -- between two known values, which contains the unknown z, and "previous" will
            -- contain the biggest z value among the known samples, left of the unknown z
            for i = 0, n - 1 do
                local k = self.keys[i + 1]
                if k < key and k > previous then
                    previous = k
                    gap = i + 1
                end
            end
        end
        local x1 = key - previous
        local x2 = self.h[gap] - x1
        local t = (-self.a[gap - 1] / 6 * (x2 + self.h[gap]) * x1 + self.values[gap])     * x2 +
                  (-self.a[gap]     / 6 * (x1 + self.h[gap]) * x2 + self.values[gap + 1]) * x1
        return t / self.h[gap]
    end,
}
---@endsection

-- Test code
--[[
local spline = SplineInterpolator:new({1, 2, 3, 4}, {30, 20, 15, 40})
for i = 0, 5.0, 0.1 do
    print(i .. ", " .. spline:getValue(i, true))
end
]]