package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.utils.hardware.MotorBuilder;

public class Intake {
    public enum IntakeState {
        START_POSITION(0.5),
        STACK_HIGH(0.3),
        STACK_MID(0.2),
        GROUND(0.1);

        public final double position;

        IntakeState(double position) {
            this.position = position;
        }
    }

    private static final double ON_POWER = 0.8;

    private final DcMotorEx intake;
    private final ServoImplEx servo;

    public Intake(HardwareMap hardwareMap) {
        intake = new MotorBuilder(hardwareMap, "intake")
                .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT)
                .build();
        servo = hardwareMap.get(ServoImplEx.class, "intake servo");
        servo.setPosition(IntakeState.START_POSITION.position);
    }

    public void on() {
        intake.setPower(ON_POWER);
    }

    public void off() {
        intake.setPower(0);
    }

    public void reverse() {
        intake.setPower(-ON_POWER);
    }

    public void reverse(double power) {
        intake.setPower(-power);
    }

    public void setServoPosition(IntakeState intakeState) {
        servo.setPosition(intakeState.position);
    }

    public void disableServo() {
        servo.setPwmDisable();
    }

    public void enableServo() {
        servo.setPwmEnable();
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        intake.setZeroPowerBehavior(behavior);
    }
}
