package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw {
    public enum ClawState {
        OPEN_INTAKE(0.04),
        OPEN_SCORE(0.075),
        CLOSED(0.02),
        SUPER_CLOSE(0.0),
        UNKNOWN(0.0);

        public final double position;

        ClawState(double position) {
            this.position = position;
        }
    }

    private final Servo clawServo;
    private ClawState clawState;

    public Claw(HardwareMap hardwareMap) {
        clawServo = hardwareMap.get(Servo.class, "claw");
        clawState = ClawState.UNKNOWN;
    }

    public void setClawState(ClawState clawState) {
        this.clawState = clawState;
        clawServo.setPosition(clawState.position);
    }

    public ClawState getClawState() {
        return clawState;
    }
}
