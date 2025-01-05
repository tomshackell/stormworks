-- Draw the UI elements for the SimpleMap: waypoints, ring/heading/trilat, buttons
-- Author: Tom Shackell

require("SimpleMap.UIButtons")
require("SimpleMap.Waypoints")
require("SimpleMap.Trilateration")

-- Inputs
IN_TOUCH_SCREEN_X = 3
IN_TOUCH_SCREEN_Y = 4
IN_GPS_X = 6
IN_GPS_Y = 7
IN_KEYPAD_A = 8
IN_KEYPAD_B = 9
IN_WP_SWITCH_DIST = 10
IN_HEADING = 11

IB_TOUCH_SCREEN_IS_PRESSED = 1
IB_KEYPAD_ENTER_PULSE = 2
IB_TRANSPONDER_DETECTED = 3

-- Outputs
ON_ZOOM = 1
ON_AUTOPILOT_WP_X = 2
ON_AUTOPILOT_WP_Y = 3
ON_HEADING = 4
ON_GPS_X = 5
ON_GPS_Y = 6
OB_AUTOPILOT_WP_VALID = 1

ZOOM_SCALE = 1.05       -- how quickly zoom in/out applies 
RING_RADIUS = 20        -- how big is the ring
TICK_LENGTH = 3         -- how long is a tick on the ring

local zoom = 2              -- current zoom level
local heading = 0           -- current heading of the boat
local gps = { 0, 0 }        -- the current location of the boat
local trilateration = nil   -- the result of performing trilateration

function onTick()
    gps = {input.getNumber(IN_GPS_X),input.getNumber(IN_GPS_Y)}
    heading = -input.getNumber(IN_HEADING) * 360

    -- check whether any UI buttons have been pressed
    local isPressed = input.getBool(IB_TOUCH_SCREEN_IS_PRESSED)
    local inputX = input.getNumber(IN_TOUCH_SCREEN_X)
    local inputY = input.getNumber(IN_TOUCH_SCREEN_Y)
    UIButtons.onTick(isPressed, inputX, inputY)

    -- update the waypoints
    local wpSwitch = input.getNumber(IN_WP_SWITCH_DIST)
    local keypad = input.getBool(IB_KEYPAD_ENTER_PULSE)
        and { input.getNumber(IN_KEYPAD_A), input.getNumber(IN_KEYPAD_B) }
        or nil
    Waypoints.onTick(wpSwitch, keypad)

    -- write the zoom & GPS to output (used by auto-pilot)
    output.setNumber(ON_ZOOM, zoom)
    output.setNumber(ON_HEADING, input.getNumber(IN_HEADING))
    output.setNumber(ON_GPS_X, gps[1])
    output.setNumber(ON_GPS_Y, gps[2])

    -- Also write the first waypoint if we have one (again for auto-pilot).
    local currWp = Waypoints.currentWaypoint()
    output.setBool(OB_AUTOPILOT_WP_VALID, currWp ~= nil)
    if currWp then
        output.setNumber(ON_AUTOPILOT_WP_X, currWp[1])
        output.setNumber(ON_AUTOPILOT_WP_Y, currWp[2])
    end

    -- run the Trilateration algorithm to locate transponder beeps
    local beep = input.getBool(IB_TRANSPONDER_DETECTED)
    trilateration = Trilaterator.onTick(beep, gps)
end

function onDraw()
    Waypoints.drawWaypoints(gps, zoom)
    drawRing(trilateration)
    Waypoints.drawUI()
    UIButtons.onDraw()
end

-- Draw the steering ring that shows heading, and possibly trilateration information
function drawRing(trilat)
    local h = screen.getHeight()
    local cx = screen.getWidth()/2
    local cy = h/2

    screen.setColor(0, 255, 0, 255)
    screen.drawCircleF(cx, cy, 1)

    screen.setColor(5, 5, 5, 255)
    screen.drawCircle(cx, cy, RING_RADIUS)

    screen.setColor(255, 0, 0, 255)
    local drawTick = function(bearing, beginRadius, length)
        local b = math.rad(bearing)
        function f(r) return cx + r*math.sin(b), cy - r*math.cos(b) end
        local x0, y0 = f(beginRadius)
        local x1, y1 = f(beginRadius + length)
        screen.drawLine(x0, y0, x1, y1)
    end

    drawTick(heading, RING_RADIUS - 4, TICK_LENGTH)
    if trilat then
    	screen.setColor(5, 5, 5, 255)
    	drawTick(trilat.altGuess, RING_RADIUS + 1, TICK_LENGTH)
    	screen.setColor(255, 255, 0, 255)
    	drawTick(trilat.bestGuess, RING_RADIUS + 1, TICK_LENGTH)
    	screen.setColor(5, 5, 5, 255)
    	local nm = math.min(trilat.distance / 1852, 999.9)
    	local text = string.format("%.1fnm", nm)
    	screen.drawTextBox(cx - 20, 2, 40, 5, text, 0, 0)
    end
end

---------------------------------------------------------------------------------------------------
--- UIButtons
---------------------------------------------------------------------------------------------------

-- set up all the buttons for zoom in/out and show/hide waypoints
UIButtons.addButton(0, 55, 8, 9, "-", false, function() zoom = zoom * ZOOM_SCALE end)
UIButtons.addButton(7, 55, 8, 9, "+", false, function() zoom = zoom / ZOOM_SCALE end)
UIButtons.addButton(88, 55, 8, 9, "w", true, Waypoints.toggleEditor)

Waypoints.init()

