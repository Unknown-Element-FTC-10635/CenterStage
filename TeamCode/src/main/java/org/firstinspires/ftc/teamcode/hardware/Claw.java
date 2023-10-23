package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw {
    public enum State {
        OPEN_INTAKE(0.04),
        OPEN_SCORE(0.75),
        CLOSED(0.02),
        SUPER_CLOSE(0.0),
        UNKNOWN(0.0);

        public final double position;

        State(double position) {
            this.position = position;
        }
    }

    private final Servo clawServo;
    private State clawState;

    public Claw(HardwareMap hardwareMap) {
        clawServo = hardwareMap.get(Servo.class, "claw");
        clawState = State.UNKNOWN;
    }

    public void setClawState(State clawState) {
        this.clawState = clawState;
        clawServo.setPosition(clawState.position);
    }

    public State getClawState() {
        return clawState;
    }
}
