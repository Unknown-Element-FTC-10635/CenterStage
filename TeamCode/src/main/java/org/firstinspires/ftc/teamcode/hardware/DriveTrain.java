package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DriveTrain {
    private final DcMotorEx frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;

    private double speedMultiplier = 0.8;
    private boolean speedToggle;

    public DriveTrain(HardwareMap hardwareMap) {
        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "front left");
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "back left");

        frontRightMotor = hardwareMap.get(DcMotorEx.class, "front right");
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor = hardwareMap.get(DcMotorEx.class, "back right");
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void driveTeleOp(float forward, float strafe, float turn) {
        frontLeftMotor.setPower(((forward - turn) + strafe) * speedMultiplier);
        backLeftMotor.setPower(((forward - turn) - strafe) * speedMultiplier);

        frontRightMotor.setPower(((forward + turn) - strafe) * speedMultiplier);
        backRightMotor.setPower(((forward + turn) + strafe) * speedMultiplier);
    }

    public void toggleSpeedMultiplier() {
        speedToggle = !speedToggle;
        if (speedToggle) {
            speedMultiplier = 1.0;
        } else {
            speedMultiplier = 0.8;
        }
    }
}
