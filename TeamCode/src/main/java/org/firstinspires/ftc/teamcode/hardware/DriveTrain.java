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

    private double speedMultiplier = 0.9;
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
        float strafe = (float) (driveController.getLeftThumbstickX() * 1.1);
        float turn = (float) (driveController.getRightThumbstickX() * 0.75);

        frontLeftMotor.setPower(((forward - strafe) * speedMultiplier) - turn);
        backLeftMotor.setPower(((forward + strafe) * speedMultiplier) - turn);

        frontRightMotor.setPower(((forward + strafe) * speedMultiplier) + turn);
        backRightMotor.setPower(((forward - strafe) * speedMultiplier) + turn);
    }

    public void toggleSpeedMultiplier() {
        speedToggle = !speedToggle;
        if (speedToggle) {
            speedMultiplier = 0.1;
        } else {
            speedMultiplier = 1;
        }
    }

    public double getSpeedMultiplier() {
        return speedMultiplier;
    }
}
