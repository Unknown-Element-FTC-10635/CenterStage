package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Delivery {
    public enum DeliveryState {
        INTAKE_PICKUP(0.175, 0.175, 0.32),
        INTAKE_HOLD(0.5, 0.5, 0.32),
        TRANSITION_2(0.68, 0.68, 0.875),
        SCORE(0.95, 0.95, 0.875),
        SCORE_AUTO(1.0, 1.0, 0.875),
        UNKNOWN(0.6, 0.6, 0.875);

        public final double pitchPositionRight;
        public final double rollPosition;

        DeliveryState(double pitchServoRight, double pitchServoLeft, double rollServo) {
            this.pitchPositionRight = pitchServoRight;
            this.rollPosition = rollServo;
        }
    }

    private final Servo pitchServoRight;
    private final Servo rollServo;

    private DeliveryState deliveryState;

    public Delivery(HardwareMap hardwareMap) {
        pitchServoRight = hardwareMap.get(Servo.class, "pitch right");
        rollServo = hardwareMap.get(Servo.class, "roll");

        deliveryState = DeliveryState.UNKNOWN;
    }

    public void setDeliveryState(DeliveryState deliveryState) {
        this.deliveryState = deliveryState;
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

    public double getRightRotationPosition() {
        return pitchServoRight.getPosition();
    }
}
