require("Tom.Math")
require("Tom.PhysicsSensor")
require("Tom.RootFinding")
require("Tom.RotMatrix")

OPT_THETA_EPSILON = 0.001    -- find the optimal theta to within these bounds (in radians)
SY_EPSILON = 0.1             -- solve to within this error in y (in meters)

WEAPON_MODELS = {
    { n="Machine Gun",      v0=800,  cd=0.005,   g=30,   tMax=300 },
    { n="Light AC",         v0=1000, cd=0.02,    g=30,   tMax=300 },
    { n="Rotary AC",        v0=1000, cd=0.01,    g=30,   tMax=300 },
    { n="Heavy AC",         v0=900,  cd=0.00489, g=30,   tMax=600 },
    { n="Battle Cannon",    v0=800,  cd=0.002,   g=30,   tMax=3600 },
    { n="Artillery Cannon", v0=700,  cd=0.001,   g=30,   tMax=3600 },
    { n="Bertha Cannon",    v0=600,  cd=0.0005,  g=30,   tMax=3600 },
}

-- This is based on https://discordapp.com/channels/357480372084408322/578586360336875520/1391403718431342653
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
    local sx = a * m.v0 * math.cos(theta) / 60
    local sy = (m.v0 * math.sin(theta) * a - (m.g/60)*(t - a)/m.cd) / 60
    return sx, sy
end

function tForTheta(m, theta, sx)
    local k = 1 - m.cd
    local q = 1 - (sx * m.cd * 60) / (k * m.v0 * math.cos(theta))
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

function onTick()
    local phys = readPhysicsSensor()
    local model = WEAPON_MODELS[input.getNumber(20)]

    local targetPos = vNew(input.getNumber(21), input.getNumber(23), input.getNumber(22)) -- map y = z-axis
    local targetVel = vNew(input.getNumber(24), input.getNumber(26), input.getNumber(25)) -- map y = z-axis
    local minElevation = turnsToRad(input.getNumber(27))
    local maxElevation = turnsToRad(input.getNumber(28))
    local highAngle = input.getBool(1)
    
    if not model or not minElevation or not maxElevation then return end 

    -- solve to hit the target (in world space)
    local targetVec = vSub(targetPos, phys.posn)
    local targetXZ = vNew(targetVec.x, 0, targetVec.z) 
    local soln = findTheta(
        model, vLen(targetXZ), targetVec.y, 
        minElevation, maxElevation, highAngle
    )

    local hasSolution = soln and soln.t <= model.tMax
    local azimuth, elevation = 0, 0
    if hasSolution then
        -- convert solution elevation angle (theta) into world-space aim vector 
        local worldAim = vNorm(vAdd(
            vMul(vNorm(targetXZ), math.cos(soln.theta)),
            vMul(vNew(0,1,0), math.sin(soln.theta))
        ))
        
        -- convert world-space vector to local vector, then get azimuth & elevation
        local localAim = rmMulV(phys.rmGlobalToLocal, worldAim)
        local aimRight = localAim.x
        local aimUp = localAim.y
        local aimFwd = localAim.z
        local aimHoriz = math.sqrt(aimFwd * aimFwd + aimRight * aimRight)
        azimuth = -radToTurns(math.atan(aimRight, aimFwd))
        elevation = radToTurns(math.atan(aimUp, aimHoriz))
    
        output.setNumber(3, vLen(targetXZ))        
        output.setNumber(4, targetVec.y)
        output.setNumber(5, soln.theta)
    end
    output.setNumber(1, azimuth)
    output.setNumber(2, elevation)     
    output.setBool(1, hasSolution)    
end
