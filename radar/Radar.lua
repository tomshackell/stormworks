require("Tom.Math")
require("Tom.RotMatrix")
require("Tom.Radar.Tracks")

INIT_V_GATE = 400                       -- maximum velocity that we can use for data association (m/s)
SWEEP_TIME = 5.25                       -- how long does it take to complete one radar sweep (s)
CELL_SIZE = INIT_V_GATE * SWEEP_TIME    -- how large is one cell (m)
MAX_RANGE = 32000                       -- maximum radar range (m)
CELL_Z_MULT = 1000000                   -- maximum number of cells in X
CELL_X_OFFS = CELL_Z_MULT / 2           -- how much to offset X coordinates

ticks = 0
orient = {}
zoom = 1

tracks = Tracks:new()

function onTick()
    orient = getOrient()
    zoom = input.getNumber(28)

    processRadarDetections(orient)

    tracks:update(orient.posn)
    ticks = ticks + 1
end

function onDraw()
    local w, h = screen.getWidth(), screen.getHeight()
    screen.drawMap(orient.posn.x, orient.posn.z, zoom)
    --screen.setColor(255, 0, 0, 255)
    for _, r in ipairs(returns) do 
        local px, py = map.mapToScreen(orient.posn.x, orient.posn.z, zoom, w, h, r.posn.x, r.posn.z)
        local alpha = lerpClamp(r.t, 0, 1, EXPIRY_TICKS, FADE_FACTOR)
        screen.setColor(255, 0, 255, alpha*255)
        screen.drawRectF(px, py, 1, 1)
    end
end

--- Retrieve the current orientation (posn + rotation matrix) from inputs
function getOrient()
    local pitch, yaw, roll = input.getNumber(16), input.getNumber(20), input.getNumber(24)
    return { 
        posn = vNew(input.getNumber(4), input.getNumber(8), input.getNumber(12)),
        rotMatrix = rmFromEuler(pitch, yaw, roll),
    }
end

-- run through radar inputs to get detections (in world space)
function processRadarDetections(orient)
    for i = 0, 7 do 
        local n = i * 4
        if input.getBool(i + 1) then
            local dist = input.getNumber(n+1)
            local azimuth = turnsToRad(input.getNumber(n+2))
            local elevation = turnsToRad(input.getNumber(n+3))            

            -- work out the position in local space (+z = fwd, +x = right, +y = up)
            local r = dist * math.cos(elevation)
            local p = vNew(r * math.sin(azimuth), dist * math.sin(elevation), r * math.cos(azimuth))

            -- transform local space position to global space relative to the radar origin and add to tracks
            tracks:addDetection(rmMulV(orient.rotMatrix, p))
        else 
            break
        end        
    end
end
