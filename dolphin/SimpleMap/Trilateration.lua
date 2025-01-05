local lastBeep = nil      -- ticks at which last beep was observed
local lastObs = nil       -- the last observation (x, y, range)
local minObsDist = 100    -- the minimum travel distance between observations
local ticks = 0           -- number of ticks observed in total
local answer = nil        -- the current answer, or nil if no answer
local distAcc = {0,0}     -- accumulated distance { sum, count }

-- A class providing a ring buffer
local RingBuff = {
    -- create a new ring-buffer of the given max size
    new = function(cls, sz)
        return {
            items={},   -- the items stored in the ring buffer
            sz=sz,      -- max size of the ring buffer
            idx=0,      -- current writing for index
            -- methods 
            write=cls.write,
            iter=cls.iter,
            clear=cls.clear,
        }
    end,
    -- write a value to the ring buffer: wrapping if needed
    write = function(self, value)
        self.items[self.idx] = value
        self.idx = (self.idx + 1) % self.sz
    end,
    -- iterate through the items in the ring-buffer, arbitrary order
    iter = function(self)
        local i = 0
        return function()
            i = i + 1
            return self.items[i-1]
        end
    end,
    -- clear the ring buffer
    clear = function(self)
        self.items = {}
        self.idx = 0
    end
}

-- the observations that were taken
local observes = RingBuff:new(50)

-- the trilaterations made from those observations
local trilats = RingBuff:new(2500)

local function vlen(v) return math.sqrt(v.x*v.x + v.y*v.y) end
local function vadd(a, b) return { x=a.x+b.x, y=a.y+b.y } end
local function vsub(a, b) return { x=a.x-b.x, y=a.y-b.y } end
local function vmul(v, f) return { x=v.x*f, y=v.y*f } end

-- calculate the bearing from one point to another
local function vbear(from, to)
    local v = vsub(to, from)
    return (450 - (math.atan(v.y, v.x) * 180 / math.pi)) % 360
end

-- calculates the absolute difference between two angles, accounting for wrapping round
local function angdiff(a, b) return 180 - math.abs(180 - math.abs(a - b) % 360) end

-- performs the trilateration algorithm
-- see: https://en.wikipedia.org/wiki/True-range_multilateration
local function trilaterate(p0, p1)
    local t = vsub(p1, p0)
    local d = vlen(t)   -- d is U on wikipedia page
    if math.abs(d) < 0.000001 then return end
    local j = (p0.r*p0.r - p1.r*p1.r + d*d) / (2*d) -- j is x 
    local k2 = p0.r*p0.r - j*j
    if k2 < 0 then return end
    local k = math.sqrt(k2)                         -- k is y
    -- now we put the j,k distances back into normal x,y coordinate space using ..
    local u = {x=t.x/d, y=t.y/d} -- normalised vector from p0 to p1
    local v = {x=u.y, y=-u.x}    -- and the right cross vector to that one 
    function posn(f) return vadd(vadd(p0, vmul(u, j)), vmul(v, k*f)) end -- p0 + u*j + v*(k*f)
    return posn(1), posn(-1)
end

-- make a guess on the bearing from here, we produce both a 'best guess' bearing (least error) and
-- an 'alternate guess' (more error). Either of these *could* turn out to be more accurate in the end.
local function makeGuess(here)
    local sums = {0, 0} -- the sums of all the bearings for the two streams
    local errs = {0, 0} -- the accumulated error on those sums
    local n = 0 -- how many items in the sum/errs
    for tri in trilats:iter() do
        -- calculate bearings for the two streams from here to the trilaterated points
        local b = { vbear(here, tri[1]), vbear(here, tri[2]) }
        if n == 0 then
            sums = b
        else
            local s = { sums[1]/n, sums[2]/n }  -- current guesses of the bearings (average)

            -- decide whether to match b[1] to s[1] (and b[2] to s[2]) or vice versa based on which 
            -- introduces the least error
            local e11 = angdiff(b[1], s[1])
            local e22 = angdiff(b[2], s[2])
            local e12 = angdiff(b[1], s[2])
            local e21 = angdiff(b[2], s[1])
            if math.min(e11, e22) < math.min(e12, e21) then
                -- prefer matching b[1] to s[1] and b[2] to s[2]
                sums = { b[1] + sums[1], b[2] + sums[2] }
                errs = { errs[1] + e11, errs[2] + e22 }
            else
                -- prefer matching [1] to s[2] and b[2] to s[1]
                sums = { b[2] + sums[1], b[1] + sums[2] }
                errs = { errs[1] + e21, errs[2] + e12 }
            end
        end
        n = n + 1
    end
    if n == 0 then return end
    -- whichever stream has lesser error is our 'best guess' and the other is the 'alternate'
    if errs[1] < errs[2] then
        return { bestGuess=sums[1]/n, altGuess=sums[2]/n, distance=here.r }
    else
        return { bestGuess=sums[2]/n, altGuess=sums[1]/n, distance=here.r }
    end
end

-- record an onservation {x,y,r}
local function observe(obs)
    -- trilaterate against every previous observation
    for other in observes:iter() do
        local p0,p1 = trilaterate(other, obs) -- trilaterate gives two answers
        if p0 then
            trilats:write({ p0, p1 }) -- store those answers in known trilaterations
        end
    end
    observes:write(obs)
    return makeGuess(obs) -- produce a guess
end

Trilaterator = {
    onTick = function(beep, gps)
        if beep then
            if lastBeep then
                -- estimate distance based on time between beeps
                -- see: https://www.reddit.com/r/Stormworks/comments/jcaygi/comment/g9bo480/
                local distance = 50.1 * (ticks - lastBeep) - 178

                -- accumulate this distance to the distance accumulator
                -- NOTE: we only output observations every `minObsDist` travelled. This avoids filling the buffer
                -- with too many (poor) measurements too close together. In between those observations we accumulate
                -- the distance measurements and take an average, which improves accuracy.
                distAcc = {distAcc[1] + distance, distAcc[2] + 1}
                local d = distAcc[1]/distAcc[2] -- current average distance in the accumulator
                local obs = {x=gps[1], y=gps[2], r=d} -- the point that was observed
                if not lastObs or vlen(vsub(lastObs, obs)) > minObsDist then -- if far enough since last observation
                    answer = observe(obs) -- then observe it and get an answer (bearing, alt-bearing and distance)
                    lastObs = obs         -- this is now the most recent observation
                    distAcc = {0,0}       -- observation so reset the distance accumulator
                end
            end
            lastBeep = ticks -- last beep was now 
        end
        ticks = ticks + 1
        if lastBeep and ticks - lastBeep > 2000 then -- about 100km, 33 seconds
            -- then turn off the trilterator because we haven't received a beep in a long time, so assume it stopped
            lastBeep = nil
            answer = nil
            observes:clear()
            trilats:clear()
        end
        return answer
    end
}
