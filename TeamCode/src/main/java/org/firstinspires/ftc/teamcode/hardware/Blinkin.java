package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.PixelColors;

public class Blinkin {
    public enum CurrentState {
        TWO_PIXELS,
        ONE_PIXELS,
        ENDGAME,
        NONE
    }

    private final RevBlinkinLedDriver leftBlinkin, rightBlinkin;
    private CurrentState currentState;

    public Blinkin(HardwareMap hardwareMap) {
        leftBlinkin = hardwareMap.get(RevBlinkinLedDriver.class, "left blinkin");
        rightBlinkin = hardwareMap.get(RevBlinkinLedDriver.class, "right blinkin");
        leftBlinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
        rightBlinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
    }

    public void setLEDColors(PixelColors intakeLeft, PixelColors intakeRight) {
        leftBlinkin.setPattern(convertToBlinkinPattern(intakeLeft));
        rightBlinkin.setPattern(convertToBlinkinPattern(intakeRight));
        currentState = CurrentState.TWO_PIXELS;
    }

    public void setEndgame() {
        leftBlinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.CONFETTI);
        currentState = CurrentState.ENDGAME;
    }

    public void strobe() {
        leftBlinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_OCEAN_PALETTE);
        currentState = CurrentState.TWO_PIXELS;
    }

    public void clear() {
        leftBlinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
        currentState = CurrentState.NONE;
    }

    private RevBlinkinLedDriver.BlinkinPattern convertToBlinkinPattern(PixelColors pixelColor) {
        switch (pixelColor) {
            case WHITE:
                return RevBlinkinLedDriver.BlinkinPattern.WHITE;
            case YELLOW:
                return RevBlinkinLedDriver.BlinkinPattern.YELLOW;
            case GREEN:
                return RevBlinkinLedDriver.BlinkinPattern.GREEN;
            case PURPLE:
                return RevBlinkinLedDriver.BlinkinPattern.VIOLET;
            default:
                return RevBlinkinLedDriver.BlinkinPattern.BLACK;
        }
    }

    public CurrentState getCurrentState() {
        return currentState;
    }
}
