require("Tom.Spline")


FUEL_USE_DATA = {
    {   rps=5.6,
        spline=SplineInterpolator:new(
            {  0.0346,     0.0419,     0.08305,    0.166,      0.276,      0.396,    0.508  },
            {  3,          3.28,       7.19,       14.18,      23.85,      34.4,     43.9   }
        ),
    },
    {   rps=8,
        spline=SplineInterpolator:new(
            {  0.0353,     0.0428,     0.0866,     0.1797,     0.3142,     0.471    },
            {  3.22,       4.3,        9.1,        20.3,       37.66,      60.35    }
        ),
    },
    {   rps=10,
        spline=SplineInterpolator:new(
            {  0.0359,     0.0436,     0.0894,     0.19,       0.3395,     0.5222   },
            {  4.7,        5.4,        15.35,      29.26,      53.65,      82.98    }
        ),
    },
    {   rps=12,
        spline=SplineInterpolator:new(
            {  0.0366,     0.0445,     0.0925,     0.199,      0.362    },
            {  6.55,       8.49,       17.08,      37.13,      67.84    }
        ),
    },
    {   rps=14,
        spline=SplineInterpolator:new(
            {  0.0375,     0.04566,    0.0958,     0.2085,     0.3854   },
            {  7.06,       9.26,       19.8,       43.95,      82.5     }
        ),
    },
    {   rps=16,
        spline=SplineInterpolator:new(
            {  0.03845,    0.04685,    0.09825,    0.2167,     0.406    },
            {  8.29,       10.28,      22.55,      52,         102.7    }
        ),
    }
}

function splinePredict(rps, fuelValve)
    local keys, values = {}, {}
    local calcFuelUse = function(predict) return predict.spline:getValue(fuelValve) end
    for _, predict in pairs(FUEL_USE_DATA) do
        table.insert(keys, predict.rps)
        table.insert(values, calcFuelUse(predict))
    end
    local spline = SplineInterpolator:new(keys, values)
    return spline:getValue(rps)
end




-- Found experimentally:
-- https://docs.google.com/spreadsheets/d/11z5Qx_K_VeQlq-J0thArYUsRatual4QL6PVy6doMo9s/edit?gid=1527351813#gid=1527351813
local function fuelFunc(a, b, c) return function(x) return a*x*x + b*x + c*x end end
FUEL_USE_PREDICTORS = {
    -- fuelUse(fuelValve) = a * fuelValve + b
    { rps=5.6, f=fuelFunc(-1.14, 87.5, -0.185) },
    { rps=8, f=fuelFunc(47.8, 107, -0.473) },
    { rps=10, f=fuelFunc(-3.1, 162, -0.828) },
    { rps=12, f=fuelFunc(5.14, 186, -0.0522) },
    { rps=14, f=fuelFunc(8.52, 213, -0.687) },
    { rps=16, f=fuelFunc(61.4, 230, -0.612) }
}
INFINITY = 1/0
NAN = -(0/0)

function predict(rps, fuelValve)
    -- find the two closest predictors to the provided RPS
    local firstBest, secondBest = { dist = INFINITY }, { dist = INFINITY }
    for _, predict in pairs(FUEL_USE_PREDICTORS) do
        local dist = math.abs(rps - predict.rps)
        if dist < firstBest.dist then
            secondBest = firstBest
            firstBest = { dist = dist, predict = predict }
        elseif dist < secondBest.dist then
            secondBest = { dist = dist, predict = predict }
        end
    end
    if not firstBest.predict or not secondBest.predict then
        return NAN
    end

    -- calculate predicted fuel use for both first and second best predictors
    local firstFuelUse = firstBest.predict.f(fuelValve)
    local secondFuelUse = secondBest.predict.f(fuelValve)

    -- then linearly interpolate between them based on rps
    return lerp(rps, firstBest.predict.rps, firstFuelUse, secondBest.predict.rps, secondFuelUse)
end

function lerp(x, minX, minY, maxX, maxY)
    return (x - minX) / (maxX - minX) * (maxY - minY) + minY
end

function smoothPredict(rps, fuelValve)
    local keys, values = {},{}
    for _,predict in pairs(FUEL_USE_PREDICTORS) do
        table.insert(keys, predict.rps)
        table.insert(values, predict.f(fuelValve))
        print(tostring(predict.rps) .. ", " .. tostring(predict.f(fuelValve)))
    end
    local spline = SplineInterpolator:new(keys, values)
    return spline:getValue(rps)
end

function onTick()
    local rps = input.getNumber(1)
    local fuelValve = input.getNumber(2)
    output.setNumber(1, predict(rps, fuelValve))
end



