package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Airplane;
import org.firstinspires.ftc.teamcode.hardware.Claw;
import org.firstinspires.ftc.teamcode.hardware.Delivery;
import org.firstinspires.ftc.teamcode.hardware.DriveTrain;
import org.firstinspires.ftc.teamcode.hardware.Intake;
import org.firstinspires.ftc.teamcode.hardware.LimitSwitch;
import org.firstinspires.ftc.teamcode.hardware.Slides;
import org.firstinspires.ftc.teamcode.hardware.Webcam;
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.PropProcessor;

import java.util.Timer;

@Autonomous(name = "BLUE (Right) - 2+0")
public class BlueLeft20 extends LinearOpMode {
    private LimitSwitch slideLimit;
    private SampleMecanumDrive driveTrain;
    private Airplane airplane;
    private Delivery delivery;
    private Intake intake;
    private Slides slides;
    private Claw claw;

    private Webcam webcam;

    private TrajectorySequence preloadDelivery, preloadDeliveryBackdrop, park;

    private ElapsedTime timer;

    @Override
    public void runOpMode() throws InterruptedException {
        slideLimit = new LimitSwitch(hardwareMap, "slide limit");
        driveTrain = new SampleMecanumDrive(hardwareMap);
        airplane = new Airplane(hardwareMap);
        delivery = new Delivery(hardwareMap);
        intake = new Intake(hardwareMap);
        slides = new Slides(hardwareMap);
        claw = new Claw(hardwareMap);

        timer = new ElapsedTime();
        timer.startTime();

        webcam = new Webcam(hardwareMap, true);
        claw.setClawState(Claw.ClawState.SINGLE_CLOSED);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Pose2d startPose = new Pose2d( 10, 62, Math.toRadians(270));
        driveTrain.setPoseEstimate(startPose);

        TrajectorySequence preloadDeliveryLeft = driveTrain.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(16, 40))
                .build();

        TrajectorySequence preloadDeliveryMiddle = driveTrain.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(12, 35), Math.toRadians(265))
                .build();

        TrajectorySequence preloadDeliveryRight = driveTrain.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(10, 40), Math.toRadians(215))
                .build();

        TrajectorySequence preloadBackboardLeftDelivery = driveTrain.trajectorySequenceBuilder(preloadDeliveryLeft.end())
                .back(10)
                .turn(Math.toRadians(-90))
                .setReversed(true)
                .lineTo(new Vector2d(50, 36))
                .back(6)
                .build();

        TrajectorySequence preloadBackboardMiddleDelivery = driveTrain.trajectorySequenceBuilder(preloadDeliveryMiddle.end())
                .back(3)
                .strafeLeft(3)
                .setReversed(true)
                .splineTo(new Vector2d(50, 34), Math.toRadians(0))
                .back(7)
                .build();

        TrajectorySequence preloadBackboardRightDelivery = driveTrain.trajectorySequenceBuilder(preloadDeliveryRight.end())
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(30, 30, DriveConstants.TRACK_WIDTH))
                .back(10)
                .turn(Math.toRadians(-45))
                .back(5)
                .setReversed(true)
                .lineTo(new Vector2d(50, 26))
                .back(8)
                .build();

        TrajectorySequence parkLeft = driveTrain.trajectorySequenceBuilder(preloadBackboardLeftDelivery.end())
                .forward(10)
                .lineTo(new Vector2d(47, 9))
                .back(6)
                .build();

        TrajectorySequence parkMiddle = driveTrain.trajectorySequenceBuilder(preloadBackboardMiddleDelivery.end())
                .forward(5)
                .lineTo(new Vector2d(45, 14))
                .back(15)
                .build();

        TrajectorySequence parkRight = driveTrain.trajectorySequenceBuilder(preloadBackboardRightDelivery.end())
                .forward(12)
                .lineTo(new Vector2d(47, 10))
                .back(10)
                .build();

        telemetry.addLine("Ready to start");
        telemetry.update();

        waitForStart();

        PropProcessor.Spikes spikePosition = webcam.propProcessor.getSpikePosition();
        webcam.stopWebcam();

        switch (spikePosition) {
            case LEFT:
                preloadDelivery = preloadDeliveryLeft;
                preloadDeliveryBackdrop = preloadBackboardLeftDelivery;
                park = parkLeft;
                break;
            case CENTER:
                preloadDelivery = preloadDeliveryMiddle;
                preloadDeliveryBackdrop = preloadBackboardMiddleDelivery;
                park = parkMiddle;
                break;
            case RIGHT:
                preloadDelivery = preloadDeliveryRight;
                preloadDeliveryBackdrop = preloadBackboardRightDelivery;
                park = parkRight;
                break;
        }

        delivery.setDeliveryState(Delivery.DeliveryState.INTAKE_HOLD);
        driveTrain.followTrajectorySequence(preloadDelivery);

        intake.reverse(0.5);
        timer.reset();
        while (!(timer.milliseconds() > 1000)) { }
        intake.off();

        delivery.setDeliveryState(Delivery.DeliveryState.TRANSITION_2);
        driveTrain.followTrajectorySequence(preloadDeliveryBackdrop);

        slides.setHeight(Slides.SlidesHeights.SECOND_LEVEL);
        delivery.setDeliveryState(Delivery.DeliveryState.SCORE_AUTO);
        timer.reset();
        while (!(slides.atTargetPosition() || timer.milliseconds() > 1000)) {
            slides.update();
        }

        timer.reset();
        while (!(timer.milliseconds() > 500)) {
            slides.update();
        }

        claw.setClawState(Claw.ClawState.OPEN_INTAKE);
        claw.depowerServo();

        timer.reset();
        while (!(timer.milliseconds() > 1000)) {
            slides.update();
        }

        delivery.setDeliveryState(Delivery.DeliveryState.INTAKE_HOLD);
        timer.reset();
        while (!(timer.milliseconds() > 1000)) {
            slides.update();
        }

        claw.repowerServo();
        slides.setHeight(Slides.SlidesHeights.BASE);
        timer.reset();
        while (!(slides.atTargetPosition() || slideLimit.isPressed() || timer.milliseconds() > 1000)) {
            slides.update();
        }

        delivery.setDeliveryState(Delivery.DeliveryState.TRANSITION_2);
        driveTrain.followTrajectorySequence(park);
    }
}
