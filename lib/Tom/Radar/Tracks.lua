require("Tom.Math")

-- Get the xi,zi coordinates for a point which is relative to radar center at (0,0,0)
local function cellCoordsForPoint(pt)
    return math.floor(pt.x / CELL_SIZE), math.floor(pt.z / CELL_SIZE)
end

-- Return the cells that are neighbouring to the given idx 
local function neighbouringCells(idx)
    local xi, zi = cellIndexToCoords(idx)
    local xoffs, zoffs = -1, -1 
    return function()
        if zoffs > 1 then return nil end 
        local ci = cellCoordsToIndex(xi + xoffs, zi + zoffs)
        xoffs = xoffs + 1 
        if xoffs > 1 then 
            zoffs = zoffs + 1 
            xoffs = -1
        end
        return ci
    end
end

-- Converts xi/zi cell coordinates to a single cell index
local function cellCoordsToIndex(xi, zi)
    return (xi + CELL_X_OFFS) + zi * CELL_Z_MULT
end

-- Converts a cell index back into xi,zi coordinates
local function cellIndexToCoords(idx)
    return (idx % CELL_Z_MULT) - CELL_X_OFFS, math.floor(idx / CELL_Z_MULT)
end

-- update the cells with a new origin
local function update(self, newOrigin)
    local dorg = vSub(newOrigin, self.lastOrigin)
    for idx, cell in pairs(self.cells) do
        for tid, track in pairs(cell.tracks) do 
                        
        end
    end    
    self.lastOrigin = newOrigin
end

-- add a new detection point (relative to the origin) to the tracks, either adding to 
-- an existing track or creating a new one.
local function addDetection(self, detPt)
    -- look for existing tracks in each of the neighbouring cells
    local xi, zi = cellCoordsForPoint(detPt)   
    local offsets = { {0,0}, {-1,0}, {1,0}, {0,-1}, {0,1}, {-1,-1}, {-1,1}, {1,-1}, {1,1} }
    for _, offs in ipairs(offsets) do
        -- attempt to add it to each of the tracks in this cell, find the best fit 
        local cell = getCell(self, xi + offs.x, zi + offs.z)
        for tid, track in pairs(cell.tracks) do

        end 
    end
end

local function getCell(self, xi, zi)
end 

Tracks = {
    new = function(cls, cellSize)
        return {
            lastOrigin = nil,
            cellSize = cellSize, 
            cells = {},

            update = update,
            addDetection = addDetection,
        }
    end,
}