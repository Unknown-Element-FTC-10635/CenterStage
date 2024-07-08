package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.utils.hardware.MotorBuilder;

public class Intake {
   public enum IntakeState {
        START_POSITION(0.1),
        STACK_HIGH(0.18),
        STACK_AUTO(0.35),
        STACK_MID(0.30),
        RYANS_SMART(0.38),
        GROUND(0.4375),
        PRELOAD(0.56);

        public final double position;

        IntakeState(double position) {
            this.position = position;
        }
    }

    private static final double ON_POWER = 0.6;
    private static final double REVERSE_POWER = -0.6;

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
        intake.setPower(REVERSE_POWER);
    }

    public void reverse(double power) {
        intake.setPower(-power);
    }

    public boolean isStalled(){
        return intake.getCurrent(CurrentUnit.AMPS) > 5;
    }

    public double getCurrent(){
        return intake.getCurrent(CurrentUnit.AMPS);
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
