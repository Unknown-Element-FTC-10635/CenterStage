package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class Claw {
    public enum ClawState {
        OPEN_AUTO(0.2),
        OPEN_INTAKE(0.32),
        OPEN_SCORE(0.36),
        CLOSED(0.49),
        SINGLE_CLOSED(0.45),
        UNKNOWN(0.0);

        public final double position;

        ClawState(double position) {
            this.position = position;
        }
    }

    private final ServoImplEx clawServo;
    private ClawState clawState;

    public Claw(HardwareMap hardwareMap) {
        clawServo = hardwareMap.get(ServoImplEx.class, "claw");
        clawState = ClawState.UNKNOWN;
    }

    public void setClawState(ClawState clawState) {
        this.clawState = clawState;
        clawServo.setPosition(clawState.position);
    }

    public ClawState getClawState() {
        return clawState;
    }

    public void depowerServo() {
        clawServo.setPwmDisable();
    }

    public void repowerServo() {
        clawServo.setPwmEnable();
    }
}
