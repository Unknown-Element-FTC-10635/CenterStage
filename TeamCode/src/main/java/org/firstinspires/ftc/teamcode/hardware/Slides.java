package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.controller.PID;
import org.firstinspires.ftc.teamcode.utils.hardware.MotorBuilder;

@Config
public class Slides {
    public enum SlidesHeights {
        BASE(1),
        PRELOAD(50),
        AUTO(100),
        SECOND_LEVEL(120),
        FOURTH_LEVEL(235),
        SIXTH_LEVEL(310),
        EIGTH_LEVEL(385),
        TENTH_LEVEL(450),
        HIGHEST_LEVEL(650);

        public final double targetPosition;

        SlidesHeights(double targetPosition) {
            this.targetPosition = targetPosition;
        }

        public static SlidesHeights levelFromInt(int num) {
            switch (num) {
                case 1:     return FOURTH_LEVEL;
                case 2:     return SIXTH_LEVEL;
                case 3:     return EIGTH_LEVEL;
                case 4:     return TENTH_LEVEL;
                case 5:     return HIGHEST_LEVEL;
                default:    return SECOND_LEVEL;
            }
        }
    }

    public static double lP = 0.03, lI = 0.0, lD = 0;
    public static double rP = 0.03, rI = 0.0, rD = 0;

    private final PID leftPIDController, rightPIDController;
    private final DcMotorEx leftExtension, rightExtension;

    private SlidesHeights currentTarget;
    private double leftError, rightError;

    private boolean pidEnabled = true;

    public Slides(HardwareMap hardwareMap) {
        leftExtension = new MotorBuilder(hardwareMap, "left ext")
                .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
                .resetEncoder()
                .build();
        rightExtension = new MotorBuilder(hardwareMap, "right ext")
                .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
                .resetEncoder()
                .setDirection(DcMotorSimple.Direction.REVERSE)
                .build();

        leftPIDController = new PID(0.0, leftExtension.getCurrentPosition(), lP, lI, lD);
        rightPIDController = new PID(0.0, rightExtension.getCurrentPosition(), rP, rI, rD);
        currentTarget = SlidesHeights.BASE;
    }

    public void setHeight(SlidesHeights targetLevel) {
        currentTarget = targetLevel;
        leftPIDController.setSetPoint(targetLevel.targetPosition);
        rightPIDController.setSetPoint(targetLevel.targetPosition);
    }

    public void setHeight(double manual) {
        leftPIDController.setSetPoint(manual);
        rightPIDController.setSetPoint(manual);
    }

    public boolean atTargetPosition() {
        leftError = (1.0 - (leftExtension.getCurrentPosition() / currentTarget.targetPosition));
        rightError = (1.0 - (rightExtension.getCurrentPosition() / currentTarget.targetPosition));
        return (Math.abs(leftError) < 0.03 + 5E-8 && Math.abs(rightError) < 0.03 + 5E-8);
    }

    public void update() {
        if (pidEnabled) {
            double leftNewPower = leftPIDController.update(leftExtension.getCurrentPosition());
            double rightNewPower = rightPIDController.update(rightExtension.getCurrentPosition());

            leftExtension.setPower(leftNewPower);
            rightExtension.setPower(rightNewPower);
        }
    }

    public void manual(double power) {
        leftExtension.setPower(power);
        rightExtension.setPower(power);
    }

    public double calculateMultiplier(double current) {
        return Math.abs((1 / (1.1 + Math.pow(Math.E, -((1 / 4.6) * (currentTarget.targetPosition - Math.abs(current)) + 0.1)))) - 0.5) + 0.1;
    }

    public void resetEncoders () {
        DcMotor.RunMode mode = leftExtension.getMode();
        leftExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftExtension.setMode(mode);
        rightExtension.setMode(mode);
    }

    public void setPidEnabled(boolean pidEnabled) {
        this.pidEnabled = pidEnabled;
    }

    public SlidesHeights getCurrentTarget() {
        return currentTarget;
    }

    public double getCurrentLeftPosition() {
        return leftExtension.getCurrentPosition();
    }

    public double getCurrentRightPosition() {
        return rightExtension.getCurrentPosition();
    }

    public double getCurrentLeftPower() {
        return leftExtension.getPower();
    }

    public double getCurrentRightPower() {
        return rightExtension.getPower();
    }

    public double getLeftError() {
        return leftError;
    }

    public double getRightError() {
        return rightError;
    }
}
