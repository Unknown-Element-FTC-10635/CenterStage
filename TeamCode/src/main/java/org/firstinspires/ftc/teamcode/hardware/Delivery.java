package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Delivery {
    public enum DeliveryState {
        INTAKE(0.0, 0.0, 0.725),
        DRIVE(0.2, 0.2, 0.725),
        SCORE(0.62, 0.62, 0.04);

        public final double pitchPositionRight;
        public final double pitchPositionLeft;
        public final double rollPosition;

        DeliveryState(double pitchServoRight, double pitchServoLeft, double rollServo) {
            this.pitchPositionRight = pitchServoRight;
            this.pitchPositionLeft = pitchServoLeft;
            this.rollPosition = rollServo;
        }
    }

    private static final double UP_THRESHOLD = 0.0;
    private static final double DOWN_THRESHOLD = 0.0;

    private boolean waitingForSafePosition = false;

    private final Servo pitchServoRight;
    private final Servo pitchServoLeft;
    private final Servo rollServo;

    private DeliveryState deliveryState;

    public Delivery(HardwareMap hardwareMap) {
        pitchServoRight = hardwareMap.get(Servo.class, "pitch right");
        pitchServoLeft = hardwareMap.get(Servo.class, "pitch left");
        rollServo = hardwareMap.get(Servo.class, "roll");

        deliveryState = DeliveryState.INTAKE;
    }

    public void setDeliveryState(DeliveryState deliveryState) {
        this.deliveryState = deliveryState;
        pitchServoLeft.setPosition(deliveryState.pitchPositionLeft);
        pitchServoRight.setPosition(deliveryState.pitchPositionRight);
        rollServo.setPosition(deliveryState.rollPosition);
        waitingForSafePosition = false;
    }

    public DeliveryState getDeliveryState() {
        return deliveryState;
    }

    public void update() {
        if (waitingForSafePosition) {
            if (deliveryState == DeliveryState.SCORE) {
                if (pitchServoRight.getPosition() < UP_THRESHOLD) {
                    rollServo.setPosition(deliveryState.rollPosition);
                    waitingForSafePosition = false;
                }
            } else {
                if (pitchServoRight.getPosition() > DOWN_THRESHOLD) {
                    rollServo.setPosition(deliveryState.rollPosition);
                    waitingForSafePosition = false;
                }
            }

        }
    }

    public boolean isWaitingForSafePosition() {
        return waitingForSafePosition;
    }

    public double getRollPosition() {
        return rollServo.getPosition();
    }
}
