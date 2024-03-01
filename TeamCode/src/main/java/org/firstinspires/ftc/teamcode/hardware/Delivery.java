package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Delivery {
    public enum DeliveryState {
        INTAKE_PICKUP   (0.10,  0.35,   0.75,   0.75),
        INTAKE_HOLD     (0.55,  0.35,   0.75,   0.75),
        TRANSITION_1    (0.62,  0.9,    0.75,   0.75),
        TRANSITION_2    (0.82,  0.9,    0.75,   0.75),
        SCORE           (0.85,  0.9,    0.39,   0.39),
        SCORE_NO_OUT    (0.89,  0.9,    0.75,   0.75),
        SCORE_PRELOAD   (0.88,  0.9,    0.75,   0.75),
        UNKNOWN         (0,     0,      0,      0);

        public final double pitchPositionRight;
        public final double rollPosition;
        public final double slidePositionLeft;
        public final double slidePositionRight;

        DeliveryState(double pitchServoRight, double rollServo, double slideServoLeft, double slideServoRight) {
            this.pitchPositionRight = pitchServoRight;
            this.rollPosition = rollServo;
            this.slidePositionLeft = slideServoLeft;
            this.slidePositionRight = slideServoRight;
        }
    }

    public enum PixelOrientation {
        NORMAL(.55),
        FLIPPED(.55),
        LEFT_45(0.25),
        RIGHT_45(0.85);

        public final double servoOrientation;

        PixelOrientation(double servoOrientation) {
            this.servoOrientation = servoOrientation;
        }
    }

    private final Servo pitchServoRight;
    private final Servo rollServo;
    private final Servo slideServoLeft, slideServoRight;
    private final Servo pixelOrientationServo;

    private DeliveryState deliveryState;
    private PixelOrientation pixelOrientation;

    public Delivery(HardwareMap hardwareMap) {
        pitchServoRight = hardwareMap.get(Servo.class, "pitch right");
        pitchServoRight.setDirection(Servo.Direction.REVERSE);
        rollServo = hardwareMap.get(Servo.class, "roll");
        slideServoLeft = hardwareMap.get(Servo.class, "slide left");
        slideServoLeft.setDirection(Servo.Direction.REVERSE);
        slideServoRight = hardwareMap.get(Servo.class, "slide right");

        pixelOrientationServo = hardwareMap.get(Servo.class, "pixel");
        pixelOrientationServo.setPosition(PixelOrientation.NORMAL.servoOrientation);

        deliveryState = DeliveryState.UNKNOWN;
    }

    public void setDeliveryState(DeliveryState deliveryState) {
        this.deliveryState = deliveryState;
        pitchServoRight.setPosition(deliveryState.pitchPositionRight);
        rollServo.setPosition(deliveryState.rollPosition);
        slideServoLeft.setPosition(deliveryState.slidePositionLeft);
        slideServoRight.setPosition(deliveryState.slidePositionRight);
    }

    public void setPixelOrientation(PixelOrientation pixelOrientation) {
        this.pixelOrientation = pixelOrientation;
        pixelOrientationServo.setPosition(pixelOrientation.servoOrientation);
    }

    public void update() {
    }

    public DeliveryState getDeliveryState() {
        return deliveryState;
    }

    public PixelOrientation getPixelOrientation() {
        return pixelOrientation;
    }

    public double getRollPosition() {
        return rollServo.getPosition();
    }

    public double getRightRotationPosition() {
        return pitchServoRight.getPosition();
    }
}
