package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.CurrentOpmode;
import org.firstinspires.ftc.teamcode.utils.hardware.GamepadEx;
import org.firstinspires.ftc.teamcode.utils.hardware.MotorBuilder;

public class DriveTrain {
    private final DcMotorEx frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;
    private final GamepadEx driveController;

    private double speedMultiplier = 0.8;
    private boolean speedToggle;

    public DriveTrain(HardwareMap hardwareMap) {
        this(hardwareMap, null);
    }

    public DriveTrain(HardwareMap hardwareMap, GamepadEx driverController) {
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

        if (CurrentOpmode.getCurrentOpmode() == CurrentOpmode.OpMode.TELEOP && driverController == null) {
            throw new NullPointerException("DriveTrain needs a gamepad for teleop");
        }
        driveController = driverController;
    }

    public void updateTeleOp() {
        float forward = driveController.getLeftThumbstickY();
        float strafe = driveController.getLeftThumbstickX();
        float turn = driveController.getRightThumbstickX();

        frontLeftMotor.setPower((forward - strafe - turn) * speedMultiplier);
        backLeftMotor.setPower((forward + strafe - turn) * speedMultiplier);

        frontRightMotor.setPower((forward + strafe + turn) * speedMultiplier);
        backRightMotor.setPower((forward - strafe + turn) * speedMultiplier);
    }

    public void toggleSpeedMultiplier() {
        speedToggle = !speedToggle;
        if (speedToggle) {
            speedMultiplier = 1.0;
        } else {
            speedMultiplier = 0.8;
        }
    }

    public double getSpeedMultiplier() {
        return speedMultiplier;
    }
}
