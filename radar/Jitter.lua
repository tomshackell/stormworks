require("Tom.Math")
require("Tom.RotMatrix")
require("Tom.RingBuffer")

MAX_SAMPLES = 3000

-- Covariance measured
--
--  6.81        -0.00035    -0.001
--  -0.00014    0.0000133   0.0000003
--  -0.00096    0.00000057  0.0000147
--

ticks = 0
orient = nil 
samples = {}
covariance = nil
target = vNew(0,0,0)
zoom = 1

function onTick()    
    orient = getOrient()
    --target = vNew(input.getNumber(28), 0, input.getNumber(29))
    zoom = input.getNumber(30)
    local reset = input.getBool(9)
    if reset then
        samples = {}
    end    

    processRadarDetections(orient)
    covariance = fitRMeas(samples)
    if covariance then 
        output.setNumber(2, covariance.meanR)
    end
    ticks = ticks + 1    
end

function onDraw()
    local w, h = screen.getWidth(), screen.getHeight()

    screen.setColor(255, 255, 255, 255)
    screen.drawRectF(0, 0, w, h)

    if covariance then 
        screen.setColor(0, 0, 0, 255)
        local sx, sy = w/3, (h - 10)/3
        for i = 0,2 do 
            for j = 0,2 do
                local idx = i * 3 + j + 1
                local v = covariance.Rmeas[idx]
                local text = string.format("%.7f", idx == 1 and v or v * 100000)
                screen.drawTextBox(j * sx, i * sy, sx, sy, text)                
            end
        end        
        screen.drawText(0, h - 10, "" .. covariance.samples)
    end
end

--- Retrieve the current orientation (posn + rotation matrix) from inputs
function getOrient()
    local pitch, yaw, roll = input.getNumber(16), input.getNumber(20), input.getNumber(24)
    return { 
        posn = vNew(input.getNumber(4), input.getNumber(8), input.getNumber(12)),
        rotMatrix = rmFromEuler(pitch, yaw, roll),  -- local to global
    }
end

-- run through radar inputs to get detections (in world space)
function processRadarDetections(orient)
    if not orient then return end
    for i = 0, 1 do 
        local n = i * 4
        if input.getBool(i + 1) then
            local dist = input.getNumber(n+1)
            local azimuth = turnsToRad(input.getNumber(n+2))
            local elevation = turnsToRad(input.getNumber(n+3))            

            -- work out the position in local space (+z = fwd, +x = right, +y = up)
            --local r = dist * math.cos(elevation)
            --local p = vNew(r * math.sin(azimuth), dist * math.sin(elevation), r * math.cos(azimuth))
    
            -- calculate in global position
            --local g = vAdd(rmMulV(orient.rotMatrix, p), orient.posn)       
            if #samples < MAX_SAMPLES then      
                table.insert(samples, { r = dist, az = azimuth, el = elevation })
            end
        else 
            break
        end        
    end
end

-- samples: array of {r=..., az=..., el=...} (az/el in radians)
function fitRMeas(samples)
    local sumR, n = 0, 0
    local sumSinAz, sumCosAz = 0,0
    local sumSinEl, sumCosEl = 0,0

    for _,s in ipairs(samples) do
        sumR = sumR + s.r
        sumSinAz = sumSinAz + math.sin(s.az)
        sumCosAz = sumCosAz + math.cos(s.az)
        sumSinEl = sumSinEl + math.sin(s.el)
        sumCosEl = sumCosEl + math.cos(s.el)
        n = n + 1
    end
    if n == 0 then return nil end

    local meanR = sumR / n
    local meanAz = math.atan(sumSinAz, sumCosAz)
    local meanEl = math.atan(sumSinEl, sumCosEl)

    -- accumulate covariance (we use unbiased 1/n here; use 1/(n-1) if you prefer sample covariance)
    local C = {0,0,0, 0,0,0, 0,0,0} -- row-major 3x3
    for _, s in ipairs(samples) do
        -- angle residuals: wrap into (-pi,pi]
        local da = s.az - meanAz
        if da > math.pi then da = da - 2*math.pi end
        if da <= -math.pi then da = da + 2*math.pi end
        local de = s.el - meanEl
        if de > math.pi then de = de - 2*math.pi end
        if de <= -math.pi then de = de + 2*math.pi end
        local dr = s.r - meanR
        -- vector [dr, da, de]
        -- outer product into C
        C[1] = C[1] + dr*dr -- 11
        C[2] = C[2] + dr*da -- 12
        C[3] = C[3] + dr*de -- 13
        C[4] = C[4] + da*dr -- 21
        C[5] = C[5] + da*da -- 22
        C[6] = C[6] + da*de -- 23
        C[7] = C[7] + de*dr -- 31
        C[8] = C[8] + de*da -- 32
        C[9] = C[9] + de*de -- 33
    end
    local invn = 1.0 / n
    for i=1,9 do C[i] = C[i]*invn end

    return {meanR=meanR, meanAz=meanAz, meanEl=meanEl, Rmeas=C, samples=n}
end