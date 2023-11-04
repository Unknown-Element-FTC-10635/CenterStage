package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Delivery {
    public enum DeliveryState {
        INTAKE_PICKUP(0.175, 0.17, 0.65),
        INTAKE_HOLD(0.3, 0.3, 0.65),
        TRANSITION_2(0.5, 0.5, 0.0),
        SCORE(0.8, 0.8, 0.05);

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

        deliveryState = DeliveryState.INTAKE_PICKUP;
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
}
