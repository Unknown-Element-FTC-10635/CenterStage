package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.controller.PID;
import org.firstinspires.ftc.teamcode.utils.hardware.MotorBuilder;

public class Slides {
    public enum SlidesHeights {
        BASE(1),
        AUTO(150),
        SECOND_LEVEL(200),
        FOURTH_LEVEL(300),
        SIXTH_LEVEL(400);

        public final double targetPosition;

        SlidesHeights(double targetPosition) {
            this.targetPosition = targetPosition;
        }

        public static SlidesHeights levelFromInt(int num) {
            switch (num) {
                case 1:     return FOURTH_LEVEL;
                case 2:     return SIXTH_LEVEL;
                default:    return SECOND_LEVEL;
            }
        }
    }

    private static final double lP = 0.09, lI = 0.0, lD = 0.005;
    private static final double rP = 0.07, rI = 0.0, rD = 0.005;

    private final PID leftPIDController, rightPIDController;
    private final DcMotorEx leftExtension, rightExtension;

    private SlidesHeights currentTarget;
    private double leftError, rightError;

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

    public boolean atTargetPosition() {
        leftError = (1.0 - (leftExtension.getCurrentPosition() / currentTarget.targetPosition));
        rightError = (1.0 - (rightExtension.getCurrentPosition() / currentTarget.targetPosition));
        return (Math.abs(leftError) < 0.03 + 5E-8 && Math.abs(rightError) < 0.03 + 5E-8);
    }

    public void update() {
        double leftNewPower = leftPIDController.update(leftExtension.getCurrentPosition()) * calculateMultiplier(leftExtension.getCurrentPosition());
        double rightNewPower = rightPIDController.update(rightExtension.getCurrentPosition()) * calculateMultiplier(rightExtension.getCurrentPosition());

        leftExtension.setPower(leftNewPower);
        rightExtension.setPower(rightNewPower);
    }

    public void manual(double power) {
        leftExtension.setPower(power);
        rightExtension.setPower(power);
    }

    public double calculateMultiplier(double current) {
        return Math.abs((1 / (1.1 + Math.pow(Math.E, -((1 / 4.6) * (currentTarget.targetPosition - Math.abs(current)) + 0.1)))) - 0.5) + 0.1;
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
