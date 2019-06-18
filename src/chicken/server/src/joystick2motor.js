// differential steering joystick algorithm
// ref.: https://www.impulseadventure.com/elec/robot-differential-steering.html

// [-128, 127]
var motorPremixL, motorPremixR
// [-128, 127]
var pivotSpeed
// [0, 1]
var pivotScale

// Threshold at which the pivot starts. 
// The larger value will let pivot action more easily to trigger [0, 127]
var pivotYLimit = 32

// [-128, 127]
var motorL, motorR

// input joystick's value joyX, joyY: [-128, 127]
function joystick2motor(joyX, joyY) {
    if (0 <= joyY) {
        motorPremixL = (0 <= joyX) ? 127 : (127 + joyX)
        motorPremixR = (0 <= joyX) ? (127 - joyX) : 127
    } else {
        motorPremixL = (0 <= joyX) ? (127 - joyX) : 127
        motorPremixR = (0 <= joyX) ? 127 : (127 + joyX)
    }

    motorPremixL *= joyY / 128.0
    motorPremixR *= joyY / 128.0

    pivotSpeed = joyX
    pivotScale = Math.abs(joyY) > pivotYLimit ? 0 : (1 - Math.abs(joyY) / pivotYLimit)

    motorL = (1 - pivotScale) * motorPremixL + pivotScale * pivotSpeed
    motorR = (1 - pivotScale) * motorPremixR + pivotScale * -pivotSpeed

    return [motorL, motorR]
}

export default joystick2motor