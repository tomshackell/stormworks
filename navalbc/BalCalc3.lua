require("Tom.RootFinding")

WEAPON_MODELS = {
    { n="Machine Gun",      v0=800,  cd=0.005,          g=30,   tMax=300 },
    { n="Light AC",         v0=1000, cd=0.02,           g=30,   tMax=300 },
    { n="Rotary AC",        v0=1000, cd=0.01,           g=30,   tMax=300 },
    { n="Heavy AC",         v0=900,  cd=0.0049,         g=30,   tMax=600 },
    { n="Battle Cannon",    v0=800,  cd=0.002,          g=30,   tMax=3600 },
    { n="Artillery Cannon", v0=700,  cd=0.001,          g=30,   tMax=3600 },
    { n="Bertha Cannon",    v0=600,  cd=0.0005,         g=30,   tMax=3600 },
}

-- Predict the position of the shell for a given theta launch angle and t time
function predict(m, theta, t)
    local k = 1 - m.cd
    local a = k * (1 - k^t) / m.cd
    local sx = a * m.v0 * math.cos(theta)
    local sy = (m.v0 * math.sin(theta) * a) - (m.g * (t - a) / m.cd)
    return sx, sy
end

-- Returns the position of the target for a given time period
function targetMotion(tgt, t) 
    return tgt.x0 + tgt.vx * t, tgt.y0 + tgt.vy * t 
end

-- Calculates the jacobian of `predict - targetMotion` as well as the two residuals
function jacobian(m, tgt, t, theta)
    local k = 1 - m.cd
    local a = k * (1 - k^t) / m.cd
    local da_dt = -(k^(t + 1) * math.log(k)) / m.cd             -- da/dt
    local df1_dt = (m.v0 * math.cos(theta) * da_dt) - tgt.vx 
    local df1_dtheta = -a * m.v0 * math.sin(theta)
    local df2_dt = (m.v0 * math.sin(theta) * da_dt) - (m.g / m.cd * (1 - da_dt)) - tgt.vy
    local df2_dtheta = a * m.v0 * math.cos(theta)
    local J = { { df1_dt, df1_dtheta }, { df2_dt, df2_dtheta } }

    local sx, sy = predict(m, theta, t)
    local tx, ty = targetMotion(tgt, t)
    return J, sx - tx, sy - ty
end

-- Calculates a seed for Newton-Raphson search by just assuming no drag or gravity.
function noDragMovingSeed(m, tgt)
    local A = tgt.vx * tgt.vx + tgt.vy * tgt.vy - m.v0 * m.v0
    local B = 2 * (tgt.x0 * tgt.vx + tgt.y0 * tgt.vy)
    local C = tgt.x0 * tgt.x0 + tgt.y0 * tgt.y0

    local eps = 1e-12
    local t = nil
    if math.abs(A) < eps then
        -- linear: B t + C = 0 so t = -C/B
        if math.abs(B) < eps then return nil end
        t = -C / B
        if t <= 0 then return nil end
    else 
        -- quadratic: A t^2 + B t + C = 0 
        local disc = B*B - 4*A*C
        if disc < 0 then return nil end

        local sqrtD = math.sqrt(disc)
        local t1 = (-B - sqrtD) / (2*A)
        local t2 = (-B + sqrtD) / (2*A)

        -- pick smallest positive root
        if t1 > 1e-9 then t = t1 end
        if t2 > 1e-9 and (not t or t2 < t) then t = t2 end
        if not t then return nil end
    end    
    local tx, ty = targetMotion(tgt, t)
    return t, math.atan2(ty, tx)
end

local m = WEAPON_MODELS[5]
local sx,sy = 2281.8, -2.52
local tgt = { x0 = 2281.8, y0 = -2.52, vx = 2, vy = 2 }
local t0, theta0 = noDragMovingSeed(m, tgt)
--local theta0, t0 = 0.071725418043078, 2.8707002792656
print("No drag: t0 = " .. tostring(t0) .. ", theta0 = " .. tostring(theta0))
local t, theta = find2DRootUsingNewton(
    function(t, theta) return jacobian(m, tgt, t, theta) end, 
    t0, theta0, 1e-5
)
print("Drag: t = " ..tostring(t)..", theta = "..tostring(theta))
local _, f1, f2 = jacobian(m, tgt, t, theta)
print("  Residuals: f1="..f1..", f2="..f2)
