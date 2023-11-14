package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.hardware.MotorBuilder;

public class Intake {
    private static final double ON_POWER = 1.0;

    private final DcMotorEx intake;

    public Intake(HardwareMap hardwareMap) {
        intake = new MotorBuilder(hardwareMap, "intake")
                .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT)
                .setDirection(DcMotorSimple.Direction.REVERSE)
                .build();
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

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        intake.setZeroPowerBehavior(behavior);
    }
}
