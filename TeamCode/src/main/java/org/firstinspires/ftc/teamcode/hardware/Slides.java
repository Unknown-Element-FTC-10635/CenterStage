package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.controller.PID;
import org.firstinspires.ftc.teamcode.utils.hardware.MotorBuilder;

public class DeliverySlides {
    public enum SlidesHeights {
        BASE(0),
        TRANSITION_STATE(100),
        SECOND_LEVEL(200),
        THIRD_LEVEL(400),
        FOURTH_LEVEL(600),
        FIFTH_LEVEL(800),
        SIXTH_LEVEL(1000);

        public final int targetPosition;

        SlidesHeights(int targetPosition) {
            this.targetPosition = targetPosition;
        }
    }

    private static final double lP = 0.0, lI = 0.0, lD = 0.0;
    private static final double rP = 0.0, rI = 0.0, rD = 0.0;

    private final PID leftPIDController, rightPIDController;
    private final DcMotorEx leftExtension, rightExtension;

    private SlidesHeights currentTarget;

    public DeliverySlides(HardwareMap hardwareMap) {
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
    }

    public void setHeight(SlidesHeights targetLevel) {
        currentTarget = targetLevel;
        leftPIDController.setSetPoint(targetLevel.targetPosition);
        rightPIDController.setSetPoint(targetLevel.targetPosition);
    }

    public boolean atTargetPosition() {
        return (leftExtension.getCurrentPosition() > currentTarget.targetPosition);
    }

    public void update() {
        double leftNewPower = leftPIDController.update(leftExtension.getCurrentPosition());
        double rightNewPower = rightPIDController.update(rightExtension.getCurrentPosition());

        leftExtension.setPower(leftNewPower);
        rightExtension.setPower(rightNewPower);
    }

    public SlidesHeights getCurrentTarget() {
        return currentTarget;
    }
}
