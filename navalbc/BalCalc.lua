require("Tom.Math")
require("Tom.PhysicsSensor")
require("Tom.RootFinding")
require("Tom.RotMatrix")

GRAVITY=30.0        -- because Stormworks is weird
DY_EPSILON=0.1      -- solve to within this error in y (in meters)

WEAPON_MODELS = {
    { n="Machine Gun",      v0=800,  k=0.005,   g=30,   tMax=5.0 },
    { n="Light AC",         v0=1000, k=0.02,    g=30,   tMax=5.0 },
    { n="Rotary AC",        v0=1000, k=0.01,    g=30,   tMax=5.0 },
    { n="Heavy AC",         v0=900,  k=0.005,   g=30,   tMax=10.0 },
    { n="Battle Cannon",    v0=800,  k=0.002,   g=30,   tMax=60.0 },
    { n="Artillery Cannon", v0=700,  k=0.001,   g=30,   tMax=60.0 },
    { n="Bertha Cannon",    v0=600,  k=0.0005,  g=30,   tMax=60.0 },
}

local Gun = {
    new = function(cls, k, v0, g, minElevation, maxElevation)
        local r = { 
            k=k, v0=v0, g=g, minElevation=minElevation, maxElevation=maxElevation,
            aimAtStatic = cls.aimAtStatic, fForTheta = cls.fForTheta, tForTheta = cls.tForTheta, predict = cls.predict,
        }
        return r
    end,
    aimAtStatic = function(self, dx, dy)
        local theta,iters = find1DRootUsingBisection(
            function(th) return self:fForTheta(dx, dy, th or 0) end,
            self.minElevation, self.maxElevation, DY_EPSILON
        )
        if theta then
            local t = self:tForTheta(dx, theta)
            return { theta=theta, t=t, iters=iters }
        else 
            return nil
        end
    end,
    fForTheta = function(self, dx, dy, theta)
        local cosTheta = math.cos(theta)
        if cosTheta <= 0 then return nil end 
        local t = self:tForTheta(dx, theta)
        if not t then return nil end 
        return -dy 
            + dx * math.tan(theta)
            + (self.g * dx) / (self.k * self.v0 * cosTheta)
            - (self.g * t) / self.k
    end,
    tForTheta = function(self, dx, theta)
        local decay = 1 - self.k * dx / (self.v0 * math.cos(theta))
        if decay <= 0 then return nil else return -math.log(decay) / self.k end
    end,
    predict = function(self, theta, t)
      local decay = 1 - math.exp(-self.k * t)
      local gOverK = self.g / self.k
      local dx = (self.v0 * math.cos(theta) / self.k) * decay
      local dy = (((self.v0 * math.sin(theta)) + gOverK) / self.k) * decay - (gOverK * t)
      return dx, dy
    end,
}

ticks = 0

function onTick()
    local phys = readPhysicsSensor()
    local model = WEAPON_MODELS[input.getNumber(20)]

    local targetPos = vNew(input.getNumber(21), input.getNumber(23), input.getNumber(22)) -- map y = z-axis
    local targetVel = vNew(input.getNumber(24), input.getNumber(26), input.getNumber(25)) -- map y = z-axis
    local minElevation = turnsToRad(input.getNumber(27))
    local maxElevation = turnsToRad(input.getNumber(28))
    
    if not model or not minElevation or not maxElevation then return end 

    -- solve to hit the target (in world space)
    local gun = Gun:new(model.cd, model.v, GRAVITY, minElevation, maxElevation)
    local targetVec = vSub(targetPos, phys.posn)
    local targetXZ = vNew(targetVec.x, 0, targetVec.z) 
    local soln = gun:aimAtStatic(vLen(targetXZ), targetVec.y)
    local hasSolution = soln and soln.t <= model.tMax
    local azimuth, elevation = 0, 0
    if hasSolution then
        -- convert solution elevation angle (theta) into world-space aim vector 
        local worldAim = vAdd(
            vMul(vNorm(targetXZ), math.cos(soln.theta)),
            vMul(vNew(0,1,0), math.sin(soln.theta))
        )
        
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
    ticks = ticks + 1
end
