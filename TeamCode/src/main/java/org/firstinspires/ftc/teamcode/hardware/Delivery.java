package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Delivery {
    public enum DeliveryState {
        INTAKE_PICKUP(0.17, 0.17, 0.65),
        INTAKE_HOLD(0.5, 0.5, 0.65),
        TRANSITION_2(0.7, 0.7, 0.0),
        SCORE(0.9, 0.9, 0.00),
        SCORE_AUTO(1.0, 1.0, 0.00),
        UNKNOWN(0.6, 0.6, 0.0);

        public final double pitchPositionRight;
        public final double pitchPositionLeft;
        public final double rollPosition;

        DeliveryState(double pitchServoRight, double pitchServoLeft, double rollServo) {
            this.pitchPositionRight = pitchServoRight;
            this.pitchPositionLeft = pitchServoLeft;
            this.rollPosition = rollServo;
        }
    }

    private final Servo pitchServoRight;
    private final Servo pitchServoLeft;
    private final Servo rollServo;

    private DeliveryState deliveryState;

    public Delivery(HardwareMap hardwareMap) {
        pitchServoRight = hardwareMap.get(Servo.class, "pitch right");
        pitchServoLeft = hardwareMap.get(Servo.class, "pitch left");
        rollServo = hardwareMap.get(Servo.class, "roll");

        deliveryState = DeliveryState.UNKNOWN;
    }

    public void setDeliveryState(DeliveryState deliveryState) {
        this.deliveryState = deliveryState;
        pitchServoLeft.setPosition(deliveryState.pitchPositionLeft);
        pitchServoRight.setPosition(deliveryState.pitchPositionRight);
        rollServo.setPosition(deliveryState.rollPosition);
    }

    public DeliveryState getDeliveryState() {
        return deliveryState;
    }

    public void update() {

    }

    public double getRollPosition() {
        return rollServo.getPosition();
    }

    public double getLeftRotationPosition() {
        return pitchServoLeft.getPosition();
    }

    public double getRightRotationPosition() {
        return pitchServoRight.getPosition();
    }
}
