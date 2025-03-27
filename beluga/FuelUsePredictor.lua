
INFINITY = 1/0
NAN = -(0/0)

-- Determined experimentally (taking a lot of time)
-- https://docs.google.com/spreadsheets/d/11z5Qx_K_VeQlq-J0thArYUsRatual4QL6PVy6doMo9s/edit?gid=1527351813#gid=1527351813
--[[
OLD_FUEL_USE_PREDICTORS = {
    -- fuelUse(fuelValve) = a * fuelValve + b
    { rps=5.6, a=86.9, b=0 },
    { rps=8, a=130, b=-1.97 },
    { rps=10, a=160, b=-0.711 },
    { rps=12, a=188, b=-0.163 },
    { rps=14, a=216, b=-0.89 },
    { rps=16, a=257, b=-2.21 }
}

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
    local calcFuelUse = function(predict) return predict.a * fuelValve + predict.b end
    local firstFuelUse = calcFuelUse(firstBest.predict)
    local secondFuelUse = calcFuelUse(secondBest.predict)

    -- then linearly interpolate between them based on rps
    return lerp(rps, firstBest.predict.rps, firstFuelUse, secondBest.predict.rps, secondFuelUse)
end
]]

FUEL_USE_PREDICTORS = {
    { throttle=0.036, d=8.81, c=-2.45, b=0.302, a=-9.64e-03 },
    { throttle=0.045, d=9.88, c=-2.68, b=0.342, a=-0.011 },
    { throttle=0.09, d=14.3, c=-3.54, b=0.511, a=-0.0168 },
    { throttle=0.195, d=18.6, c=-3.59, b=0.715, a=-0.0244 },
    { throttle=0.35, d=9.95, c=1.35, b=0.525, a=-0.021 },
    { throttle=0.5, d=-15.6, c=11.8, b=-0.215, a=-9.39e-04 },
}

function lerp(x, minX, minY, maxX, maxY)
    return (x - minX) / (maxX - minX) * (maxY - minY) + minY
end

function predict(rps, fuelValve)
    -- find the two closest predictors to the provided RPS
    local firstBest, secondBest = { dist = INFINITY }, { dist = INFINITY }
    for _, predict in pairs(FUEL_USE_PREDICTORS) do
        local dist = math.abs(fuelValve - predict.throttle)
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
    local calcFuelUse = function(predict)
        return predict.a*rps*rps*rps + predict.b*rps*rps + predict.c*rps + predict.d
    end
    local firstFuelUse = calcFuelUse(firstBest.predict)
    local secondFuelUse = calcFuelUse(secondBest.predict)

    -- then linearly interpolate between them based on throttle
    local throttle1, throttle2 = firstBest.predict.throttle, secondBest.predict.throttle
    return lerp(fuelValve, throttle1, firstFuelUse, throttle2, secondFuelUse)
end

function onTick()
    local rps = input.getNumber(1)
    local fuelValve = input.getNumber(2)
    output.setNumber(1, predict(rps, fuelValve))
end



