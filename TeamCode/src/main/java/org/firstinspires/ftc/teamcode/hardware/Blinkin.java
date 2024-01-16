package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.PixelColors;

public class Blinkin {
    public enum CurrentState {
        TWO_PIXELS,
        ONE_PIXELS,
        NONE
    }

    private final RevBlinkinLedDriver blinkin;
    private CurrentState currentState;

    public Blinkin(HardwareMap hardwareMap) {
        blinkin = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
    }

    public void setOnePixel(PixelColors color) {
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
        currentState = CurrentState.ONE_PIXELS;
    }

    public void setTwoPixel(PixelColors intakeLeft, PixelColors intakeRight) {
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
        currentState = CurrentState.TWO_PIXELS;
    }

    public void clear() {
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
        currentState = CurrentState.NONE;
    }

    public CurrentState getCurrentState() {
        return currentState;
    }
}
