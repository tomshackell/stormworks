-- Draws the basic map with zoom, and any radar contacts.
-- Author: Tom Shackell

-- Numbers: 1 to 24, GPS X/Y/Z for all 8 contacts
IN_ZOOM = 25
IN_RADAR_ANGLE = 26
IN_GPS_X = 27
IN_GPS_Y = 28
-- Bool: 1 to 8, contact is valid for all 8 contacts
IB_RADAR_ON = 9

EXPIRY_TICKS = 150      -- how many ticks for a contact to expire
FADE_FACTOR = 0.75      -- how much a contact fades at full expiry time 


zoom = 2                -- current zoom level
gps = nil               -- current GPS coordinate to display the map at 
ticks = 0               -- how many ticks have we seen 
isRadarOn=false         -- whether the radar is on
contacts = {}           -- stored radar contacts 
nextContactID = 0       -- the next ID to use for a new radar contact

function onTick()
	zoom = input.getNumber(IN_ZOOM)
    gps = {input.getNumber(IN_GPS_X), input.getNumber(IN_GPS_Y)}
    isRadarOn = input.getBool(IB_RADAR_ON)
	radarAngle = input.getNumber(IN_RADAR_ANGLE)

    -- read the radar contacts from the input, each is created as a new contact
	for i = 0,7 do
		local active = input.getBool(i+1)
		if active then
			local x = input.getNumber(i*3+1)
			local y = input.getNumber(i*3+2)
			local z = input.getNumber(i*3+3)
			contacts[nextContactID] = {posn={x,y,z}, time=ticks}
			nextContactID = nextContactID + 1
		end
	end

    -- if any existing contacts have expired then remove them
	for id,contact in pairs(contacts) do
		if ticks - contact.time > EXPIRY_TICKS then
			contacts[id] = nil
		end
	end
	ticks = ticks + 1
end

function onDraw()
	local sc = screen
	local w,h = sc.getWidth(), sc.getHeight()

    -- draw the base map, zoomed as appropriate
    sc.drawMap(gps[1], gps[2], zoom)

    -- if the radar is one then draw all the radar contacts onto the screen
    if isRadarOn then
        screen.setColor(255, 0, 0, 255)
		for _,con in pairs(contacts) do 
			local px,py = map.mapToScreen(gps[1], gps[2], zoom, w, h, con.posn[1], con.posn[2])
			local alpha = math.min(1 - ((con.time/EXPIRY_TICKS) - FADE_FACTOR)/(1 - FADE_FACTOR), 1)
			screen.setColor(255, 0, 255, alpha*255)
			screen.drawRectF(px,py,1,1)
		end
    end
end
