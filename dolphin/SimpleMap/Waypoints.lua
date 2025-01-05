require("SimpleMap.UIButtons")

local NUM_WPS = 8               -- the total number of supported waypoints
local showEditor = false        -- should the waypoint editor be shown
local currentWP = 1             -- the current waypoint (for autopilot)
local inputWP = 0               -- the input waypoint (where the keypad will set) or 0 for none
local waypoints = {}            -- all the waypoints
local ticks = 0                 -- total number of ticks since start
local gps = {0, 0}              -- the gps coordinates of the boat

-- creates a new empty waypoint entry
local function emptyWP()
    return {
        posn = {0, 0},
        active = false,

        -- calculates the distance to this waypoint (in nautical miles)
        dist = function(self)
            return math.sqrt((self.posn[1] - gps[1])^2 + (self.posn[2] - gps[2])^2) / 1852
        end
    }
end

-- draw the selector array: a button without a border, but only show it against the current WP
local function drawSelector(self, id)
	if id == currentWP then
        self:defaultDraw(true) -- true = no border
    end
end

-- draw the input field for the WP, this is a button but shows the distance to the WP 
local function drawInput(self, id)
    if id == inputWP and (ticks//20)%2==0 then
        self.text = ""  -- flash text on/off to show editable
    else
        local wp = waypoints[id]
    	if wp.active then
            self.text = string.format("%.1fnm", wp:dist())
    	else
    		self.text = "-------"
        end
    end
    self:defaultDraw() -- continue to draw as a normal button
end

-- move the given waypoint up or down
local function moveWP(id, dir)
    local other = id+dir
    if other <= 0 or other > NUM_WPS then
        return
    end

    -- swap with the waypoint above/below
    local tmp = waypoints[other]
    waypoints[other] = waypoints[id]
    waypoints[id] = tmp

    -- update currentWP if we just moved that
    if currentWP == id then
        currentWP = other
    elseif currentWP == other then
        currentWP = id
    end

    -- update inputWP if we just moved that
    if inputWP == id then
        inputWP = other
    elseif inputWP == other then
        inputWP = id
    end
end

-- remove the given waypoint from the waypoints (shifts those after up)
local function removeWP(id)
    if currentWP > id then
        currentWP = currentWP - 1
    end
    if inputWP == id then
        inputWP = 0
    elseif inputWP > id then
        inputWP = inputWP - 1
    end
    for i = id, NUM_WPS-1 do
        waypoints[i] = waypoints[i+1]
    end
    waypoints[NUM_WPS] = emptyWP()
end

Waypoints = {
    init = function()
        for i = 1, NUM_WPS do
            waypoints[i] = emptyWP()

            -- add the buttons for the waypoints
            local y = (i-1)*8
            local button = function(x, w, text, onPress, draw)
                UIButtons.addButton(x, y-1, w, 9, text, true, onPress,
                    function()
                        return showEditor  -- only enabled if we are showing waypoint editing screen
                    end,
                    draw,
                    function()
                        if i == currentWP then
                            screen.setColor(255, 255, 0, 255) -- highlight current waypoint in yellow
                        end
                    end
                )
            end
            local setCurrent = function() currentWP = i end
            local setInput = function() inputWP = (inputWP == i) and 0 or i end
            button(14, 8, ">", setCurrent, function(b) drawSelector(b, i) end)
            button(21, 8, i, setCurrent, nil)
            button(28, 40, "", setInput, function(b) drawInput(b,i) end)
            button(67, 8, "^", function() moveWP(i, -1) end, nil)
            button(74, 8, "v", function() moveWP(i, 1) end, nil)
            button(81, 8, "x", function() removeWP(i) end, nil)
        end
    end,

    onTick = function(switchDist, keypad)
        ticks = ticks + 1

        -- check for keypad input into a waypoint 
        if keypad and inputWP ~= 0 then
            waypoints[inputWP].posn = keypad
            waypoints[inputWP].active = true
            if currentWP > NUM_WPS and inputWP == 1 then
                currentWP = 1
            end
            inputWP = 0
        end

        -- advance the current waypoint if we have got close enough
        while currentWP <= NUM_WPS do
            local wp = waypoints[currentWP]
            if wp:dist() < switchDist or not wp.active then
                currentWP = currentWP + 1
            else
                break
            end
        end
    end,

    drawWaypoints = function(gpsIn, zoom)
        local w, h = screen.getWidth(), screen.getHeight()
        gps = gpsIn

        -- draw the waypoint markers/lines to the screen
        local lastX, lastY = w/2, h/2   -- start the line from current position
        local points = {}
        local numPoints = 0
        local currentDist = nil
        for i = currentWP, NUM_WPS do
            local wp = waypoints[i]
            if wp.active then
                -- draw the line from the last point to this one
                local px,py = map.mapToScreen(gps[1], gps[2], zoom, w, h, wp.posn[1], wp.posn[2])
                screen.setColor(20, 20, 20, 255)
                screen.drawLine(lastX, lastY, px, py)
                lastX,lastY = px,py -- this is the new last point

                -- if this is the current WP then remember the distance
                if i == currentWP then
                    currentDist = string.format("%.1fnm", wp:dist())
                end

                -- save the point for drawing later
                numPoints = numPoints + 1
                points[numPoints] = { px, py }
            end
        end

        -- then draw the waypoint dots over the lines
        for _, p in pairs(points) do
            screen.setColor(0, 0, 255, 255)
            screen.drawRectF(p[1], p[2], 1, 1)
        end

        -- finally draw the distance to the current waypoint
        if currentDist then
            screen.setColor(5, 5, 5, 255)
            screen.drawTextBox(15, 57, 73, 7, currentDist, 0, 0)
        end
    end,

    drawUI = function()
        -- draw the background for the waypoint editor, if we are showing that 
        if showEditor then
            screen.setColor(0,0,0, 255)
            screen.drawRectF(14,0,74,64)
            screen.setColor(50,50,50, 255)
            screen.drawRect(14,-1,74,64)
        end
    end,

    -- toggle showing the waypoint editor
    toggleEditor = function()
        showEditor = not showEditor
    end,

    -- Return the current active waypoint in the list (or nil)
    currentWaypoint = function()
        for i = currentWP, NUM_WPS do
            if waypoints[i].active then
                return waypoints[i].posn
            end
        end
    end,
}
