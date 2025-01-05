local buttons = {}
local nextButton = 1

-- draw a single button, possibly with a border 
local function drawButton(b, noBorder)
    if not b:en() then
        return
    end
    screen.setColor(50, 50, 50, 255)
    if not noBorder then
        if b.pressed then
            -- invert colors for a pressed button
            screen.drawRectF(b.rect[1], b.rect[2], b.rect[3]-1, b.rect[4]-1)
            screen.setColor(255, 255, 255, 255)
        else
            screen.drawRect(b.rect[1], b.rect[2], b.rect[3]-1, b.rect[4]-1)
        end
    end
    b:preText()
    screen.drawText(b.rect[1]+2, b.rect[2]+2, b.text)
end

UIButtons = {
    -- create a new button with the given properties
    -- x,y,w,h the rectange for the button 
    -- text the text to put into the button
    -- debounce if true the button will implement 'debouncing' to prevent being pressed twice 
    -- onPress the function to run when the button has been pressed, or nil for no action
    -- en a function that says when the button is shown/pressable, or nil for always
    -- draw a function that draws the button, or nil for the default drawButton
    -- preText a function to run before rendering the text in the button, or nil for no action
    addButton = function(x, y, w, h, text, debounce, onPress, en, draw, preText)
        local nop = function() return true end
        buttons[nextButton] = {
            rect = {x,y,w,h},
            text = text or "",
            debounce = debounce,
            onPress = onPress or nop,
            en = en or nop,
            draw = draw or drawButton,
            preText = preText or nop,
            wasPressed = false,
            pressed = false,
            defaultDraw = drawButton,
        }
        nextButton = nextButton + 1
    end,

    -- Called every tick
    onTick = function(isPressed, inputX, inputY)
        inRect = function(x, y, w, h)
            return inputX > x and inputY > y and inputX < x+w and inputY < y+h
        end

        -- check for buttons being pressed    
        for _,b in pairs(buttons) do
            b.pressed = isPressed and b:en() and
                inRect(b.rect[1]+1,b.rect[2]+1,b.rect[3]-2,b.rect[4]-2)

            if b.pressed then
                if not b.debounce or not b.wasPressed then
                    b:onPress()
                end
                b.wasPressed = true
            else
                b.wasPressed = false
            end
        end
    end,

    onDraw = function()
        for _, b in pairs(buttons) do
            b:draw(false)
        end
    end,
}
