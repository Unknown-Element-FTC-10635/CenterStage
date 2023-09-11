package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.hardware.MotorBuilder;

public class DriveTrain {
    private final DcMotorEx frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;

    private double speedMultiplier = 0.8;
    private boolean speedToggle;

    public DriveTrain(HardwareMap hardwareMap) {
        frontLeftMotor = new MotorBuilder(hardwareMap, "front left")
                .resetEncoder()
                .build();
        backLeftMotor = new MotorBuilder(hardwareMap, "back left")
                .resetEncoder()
                .build();

        frontRightMotor = new MotorBuilder(hardwareMap, "front right")
                .setDirection(DcMotorSimple.Direction.REVERSE)
                .resetEncoder()
                .build();
        backRightMotor = new MotorBuilder(hardwareMap, "back right")
                .setDirection(DcMotorSimple.Direction.REVERSE)
                .resetEncoder()
                .build();
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
