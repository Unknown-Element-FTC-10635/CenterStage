package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Delivery {
    public enum DeliveryState {
        INTAKE_PICKUP   (0.09,  0.34,   0.675,  0.675),
        INTAKE_HOLD     (0.5,   0.34,   0.675,  0.675),
        TRANSITION_2    (0.71,  0.88,   0.675,  0.675),
        SCORE           (0.8,   0.88,   0.43,   0.43),
        SCORE_AUTO      (0.85,  0.88,   0.43,   0.43),
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
        rollServo = hardwareMap.get(Servo.class, "roll");
        slideServoLeft = hardwareMap.get(Servo.class, "slide left");
        slideServoLeft.setDirection(Servo.Direction.REVERSE);
        slideServoRight = hardwareMap.get(Servo.class, "slide right");

        deliveryState = DeliveryState.UNKNOWN;
    }

    public void setDeliveryState(DeliveryState deliveryState) {
        this.deliveryState = deliveryState;
        pitchServoRight.setPosition(deliveryState.pitchPositionRight);
        pitchServoRight.setDirection(Servo.Direction.REVERSE);
        rollServo.setPosition(deliveryState.rollPosition);
        slideServoLeft.setPosition(deliveryState.slidePositionLeft);
        slideServoRight.setPosition(deliveryState.slidePositionRight);
    }

    public DeliveryState getDeliveryState() {
        return deliveryState;
    }

    public void update() {

    }

    public double getRollPosition() {
        return rollServo.getPosition();
    }

    public double getRightRotationPosition() {
        return pitchServoRight.getPosition();
    }
}
