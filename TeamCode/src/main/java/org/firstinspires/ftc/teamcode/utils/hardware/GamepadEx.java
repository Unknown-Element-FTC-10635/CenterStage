package org.firstinspires.ftc.teamcode.utils.hardware;

import com.qualcomm.robotcore.hardware.Gamepad;

public class GamepadEx {
    public enum Buttons {
        CROSS,
        CIRCLE,
        TRIANGLE,
        SQUARE,
        DPAD_DOWN,
        DPAD_RIGHT,
        DPAD_UP,
        DPAD_LEFT,
        SHARE,
        OPTION,
        PLAYSTATION,
        BUMPER_LEFT,
        BUMPER_RIGHT,
        L3,
        R3,
        TOUCHPAD
    }

    private final Gamepad baseGamepad;
    private final Gamepad currentGamepad;
    private final Gamepad previousGamepad;

    public GamepadEx(Gamepad gamepad) {
        currentGamepad = new Gamepad();
        previousGamepad = new Gamepad();

        baseGamepad = gamepad;
    }

    public boolean risingEdgeOf(Buttons button) {
        switch (button) {
            case CROSS:         return (currentGamepad.cross && !previousGamepad.cross);
            case CIRCLE:        return (currentGamepad.circle && !previousGamepad.circle);
            case TRIANGLE:      return (currentGamepad.triangle && !previousGamepad.triangle);
            case SQUARE:        return (currentGamepad.square && !previousGamepad.square);
            case DPAD_DOWN:     return (currentGamepad.dpad_down && !previousGamepad.dpad_down);
            case DPAD_RIGHT:    return (currentGamepad.dpad_right && !previousGamepad.dpad_right);
            case DPAD_UP:       return (currentGamepad.dpad_up && !previousGamepad.dpad_up);
            case DPAD_LEFT:     return (currentGamepad.dpad_left && !previousGamepad.dpad_left);
            case SHARE:         return (currentGamepad.share && !previousGamepad.share);
            case OPTION:        return (currentGamepad.options && !previousGamepad.options);
            case PLAYSTATION:   return (currentGamepad.ps && !previousGamepad.ps);
            case BUMPER_LEFT:   return (currentGamepad.left_bumper && !previousGamepad.left_bumper);
            case BUMPER_RIGHT:  return (currentGamepad.right_bumper && !previousGamepad.right_bumper);
            case L3:            return (currentGamepad.left_stick_button && !previousGamepad.left_stick_button);
            case R3:            return (currentGamepad.right_stick_button && !previousGamepad.right_stick_button);
            case TOUCHPAD:      return (currentGamepad.touchpad && !previousGamepad.touchpad);
            // Can never reach
            default:            return false;
        }
    }

    public boolean fallingEdgeOf(Buttons button) {
        switch (button) {
            case CROSS:         return (!currentGamepad.cross && previousGamepad.cross);
            case CIRCLE:        return (!currentGamepad.circle && previousGamepad.circle);
            case TRIANGLE:      return (!currentGamepad.triangle && previousGamepad.triangle);
            case SQUARE:        return (!currentGamepad.square && previousGamepad.square);
            case DPAD_DOWN:     return (!currentGamepad.dpad_down && previousGamepad.dpad_down);
            case DPAD_RIGHT:    return (!currentGamepad.dpad_right && previousGamepad.dpad_right);
            case DPAD_UP:       return (!currentGamepad.dpad_up && previousGamepad.dpad_up);
            case DPAD_LEFT:     return (!currentGamepad.dpad_left && previousGamepad.dpad_left);
            case SHARE:         return (!currentGamepad.share && previousGamepad.share);
            case OPTION:        return (!currentGamepad.options && previousGamepad.options);
            case PLAYSTATION:   return (!currentGamepad.ps && previousGamepad.ps);
            case BUMPER_LEFT:   return (!currentGamepad.left_bumper && previousGamepad.left_bumper);
            case BUMPER_RIGHT:  return (!currentGamepad.right_bumper && previousGamepad.right_bumper);
            case L3:            return (!currentGamepad.left_stick_button && previousGamepad.left_stick_button);
            case R3:            return (!currentGamepad.right_stick_button && previousGamepad.right_stick_button);
            case TOUCHPAD:      return (!currentGamepad.touchpad && previousGamepad.touchpad);
            // Can never reach
            default:            return false;
        }
    }

    public float getLeftThumbstickX() {
        return currentGamepad.left_stick_x;
    }

    public float getLeftThumbstickY() {
        return currentGamepad.left_stick_y;
    }

    public float getRightThumbstickX() {
        return currentGamepad.right_stick_x;
    }

    public float getRightThumbstickY() {
        return currentGamepad.right_stick_y;
    }

    public boolean leftTriggerPressedLiterallyAtAllRisingEdge() {
        return (currentGamepad.left_trigger > 0.1) && !(previousGamepad.left_trigger > 0.1);
    }

    public boolean leftTriggerPressedLiterallyAtAllFallingEdge() {
        return !(currentGamepad.left_trigger > 0.1) && (previousGamepad.left_trigger > 0.1);
    }

    public void update() {
        previousGamepad.copy(currentGamepad);
        currentGamepad.copy(baseGamepad);
    }
}
