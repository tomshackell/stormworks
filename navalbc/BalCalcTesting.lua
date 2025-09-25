OPT_THETA_EPSILON = 0.001    -- find the optimal theta to within these bounds (in radians)
SY_EPSILON = 0.1             -- solve to within this error in y (in meters)

WEAPON_MODELS = {
    { n="Machine Gun",      v0=800,  cd=0.005,          g=30,   tMax=300 },
    { n="Light AC",         v0=1000, cd=0.02,           g=30,   tMax=300 },
    { n="Rotary AC",        v0=1000, cd=0.01,           g=30,   tMax=300 },
    { n="Heavy AC",         v0=900,  cd=0.0049,         g=30,   tMax=600 },
    { n="Battle Cannon",    v0=800,  cd=0.002,          g=30,   tMax=3600 },
    { n="Artillery Cannon", v0=700,  cd=0.001,          g=30,   tMax=3600 },
    { n="Bertha Cannon",    v0=600,  cd=0.0005,         g=30,   tMax=3600 },
}

function find1DRootUsingBisection(f, minX, maxX, epsilon)
    if not minX or not maxX then 
        return nil, 0 
    end 
    local minY, maxY = f(minX), f(maxX)
    if not minY or not maxY or minY * maxY > 0 then
        return nil, 0 -- no solution
    end
    local lowX, lowY, highX, highY = minX, minY, maxX, maxY
    local iters = 0
    while math.abs(highY - lowY) > epsilon do
        local midX = (lowX + highX) / 2
        local midY = f(midX)
        if midY * lowY > 0 then -- mid is same sign as low, so this refines low
            lowX, lowY = midX, midY
        else 
            highX, highY = midX, midY
        end
        iters = iters + 1
    end 
    return (lowX + highX) / 2, iters
end

function ternarySearch(f, minX, maxX, epsilon)
    local loX, hiX = minX, maxX 
    while math.abs(hiX - loX) > epsilon do
        local third = (hiX - loX) / 3
        local x1, x2 = loX + third, hiX - third
        local y1, y2 = f(x1), f(x2)
        if y1 <= y2 then loX = x1 end
        if y1 >= y2 then hiX = x2 end
    end
    return (hiX + loX) / 2
end

-- This is based on
-- By smithy3141
--  
--  Velocity at some time `t` is given by `V = I * k^t` the position is just the sum of these velocities
--
--  I_x = 636 --Initial Velocity
--  I_y = 636
--  t = 120 --time in ticks, 60 = one second
--  drag = 0.009
--
--  k = 1-drag
--
--  V_x = I_x*k^t --Velocity at a given time
--  V_y = I_x*k^t-0.5*(1-k^t)/drag
--  a = k*(1-k^t)/drag
--  S_x = a*I_x/60 --horizontal position
--  S_y = (I_y*a-(g/60)*(t-a)/drag)/60 --vertical position
--
-- This is a discrete version of the normal linear drag equations. 
-- NOTE: the typical continuous ones don't give the same answer.
function predict(m, theta, t)
    local k = 1 - m.cd
    local a = k * (1 - k^t) / m.cd
    local sx = a * m.v0 * math.cos(theta)
    local sy = (m.v0 * math.sin(theta) * a - m.g * (t - a)/m.cd)
    return sx, sy
end

function tForTheta(m, theta, sx)
    local k = 1 - m.cd
    local q = 1 - (sx * m.cd) / (k * m.v0 * math.cos(theta))
    if q <= 0 then return nil end  -- No solution possible
    return math.log(q) / math.log(k)
end

function findTheta(m, sx, sy, minTheta, maxTheta, highAngle)
    -- gives the error in sy for a given theta angle
    local syErrForTheta = function(theta) 
        local t = tForTheta(m, theta, sx)
        if not t then return -math.huge end -- fell short: theta is too high
        local _, asy = predict(m, theta, t)
        return asy - sy
    end
    
    -- use ternary search to find the maximal syError, giving us the theta with the longest reach
    local optTheta = ternarySearch(syErrForTheta, minTheta, maxTheta, OPT_THETA_EPSILON)

    -- then use that for the low/high theta bounds for either a high or low angle shot
    local lowTheta = highAngle and optTheta or minTheta 
    local highTheta = highAngle and maxTheta or optTheta 
    
    -- find the theta that gives syErrForTheta as close to zero as possible
    local theta = find1DRootUsingBisection(syErrForTheta, lowTheta, highTheta, SY_EPSILON)
    return theta and { theta = theta, t = tForTheta(m, theta, sx) } or nil
end

local m = WEAPON_MODELS[5]
local sx,sy = 2281.8, -2.52
local soln = findTheta(m, sx, sy, -math.pi/2, math.pi/2, false)
print("theta = " .. soln.theta .. ", t=" .. soln.t)

local realSx = sx - 10.0
local cd = find1DRootUsingBisection(
  function(cd)
    m.cd = cd
    local gotTheta = findTheta(m, realSx, sy, -math.pi/2, math.pi/2, false)
    return gotTheta and soln.theta - gotTheta.theta or -math.huge
  end,
  0.0015, 0.0025, 0.0001
)
print("cd = " .. cd)

