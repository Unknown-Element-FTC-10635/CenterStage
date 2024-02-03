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
        START_POSITION(0.1),
        STACK_HIGH(0.18),
        STACK_AUTO(0.22),
        STACK_MID(0.27),
        GROUND(0.41);

        public final double position;

        IntakeState(double position) {
            this.position = position;
        }
    }

    private static final double ON_POWER = 1.0;

    private final DcMotorEx intake;
    private final ServoImplEx servo;

    public Intake(HardwareMap hardwareMap) {
        intake = new MotorBuilder(hardwareMap, "intake")
                .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT)
                .build();
        servo = hardwareMap.get(ServoImplEx.class, "intake servo");
    }

    public void on() {
        intake.setPower(ON_POWER);
    }

    public void on(double power) {
        intake.setPower(power);
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
