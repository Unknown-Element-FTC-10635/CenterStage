package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Delivery {
    public enum DeliveryState {
        INTAKE_PICKUP   (0.14,  0.36,   0.72,   0.72),
        INTAKE_HOLD     (0.55,  0.36,   0.72,   0.72),
        TRANSITION_1    (0.63,  0.9,    0.72,   0.72),
        TRANSITION_2    (0.82,  0.9,    0.72,   0.72),
        SCORE           (0.86,  0.9,    0.39,   0.39),
        SCORE_PRELOAD   (0.98,  0.9,    0.72,   0.72),
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

    private final Servo pitchServoRight;
    private final Servo rollServo;
    private final Servo slideServoLeft, slideServoRight;

    private DeliveryState deliveryState;

    public Delivery(HardwareMap hardwareMap) {
        pitchServoRight = hardwareMap.get(Servo.class, "pitch right");
        pitchServoRight.setDirection(Servo.Direction.REVERSE);
        rollServo = hardwareMap.get(Servo.class, "roll");
        slideServoLeft = hardwareMap.get(Servo.class, "slide left");
        slideServoLeft.setDirection(Servo.Direction.REVERSE);
        slideServoRight = hardwareMap.get(Servo.class, "slide right");

        deliveryState = DeliveryState.UNKNOWN;
    }

    public void setDeliveryState(DeliveryState deliveryState) {
        this.deliveryState = deliveryState;
        pitchServoRight.setPosition(deliveryState.pitchPositionRight);
        rollServo.setPosition(deliveryState.rollPosition);
        slideServoLeft.setPosition(deliveryState.slidePositionLeft);
        slideServoRight.setPosition(deliveryState.slidePositionRight);
    }

    public void update() {
    }

    public DeliveryState getDeliveryState() {
        return deliveryState;
    }

    public double getRollPosition() {
        return rollServo.getPosition();
    }

    public double getRightRotationPosition() {
        return pitchServoRight.getPosition();
    }
}
