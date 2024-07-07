package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Claw;
import org.firstinspires.ftc.teamcode.hardware.Delivery;
import org.firstinspires.ftc.teamcode.hardware.Intake;
import org.firstinspires.ftc.teamcode.hardware.Webcam;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.PropProcessor;

@Disabled
@Autonomous(name = "BLUE (Backboard) Outside - 2+0", group = "blue")
public class BlueLeft20Outside extends LinearOpMode {
    private SampleMecanumDrive driveTrain;
    private Delivery delivery;
    private Intake intake;
    private Claw claw;

    private Webcam webcam;

    private TrajectorySequence preloadDelivery, preloadDeliveryBackdrop, park;

    private ElapsedTime timer;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Building hardware");
        telemetry.update();

        driveTrain = new SampleMecanumDrive(hardwareMap);
        delivery = new Delivery(hardwareMap);
        intake = new Intake(hardwareMap);
        claw = new Claw(hardwareMap);

        telemetry.addLine("Creating timers");
        telemetry.update();

        timer = new ElapsedTime();
        timer.startTime();

        telemetry.addLine("Initializing robot");
        telemetry.update();

        PropProcessor processor = new PropProcessor(true);
        webcam = new Webcam(hardwareMap, processor, "webcam");
        claw.setClawState(Claw.ClawState.SINGLE_CLOSED);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("Building paths");
        telemetry.update();

        Pose2d startPose = new Pose2d( 9, 62, Math.toRadians(270));
        driveTrain.setPoseEstimate(startPose);


        TrajectorySequence preloadDeliveryLeft = driveTrain.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(14, 53))
                .lineToLinearHeading(new Pose2d(22,36, Math.toRadians(260)))
                .back(5)
                .build();

        TrajectorySequence preloadDeliveryMiddle = driveTrain.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(13, 30))
                .back(7)
                .build();

        TrajectorySequence preloadDeliveryRight = driveTrain.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(15, 52))
                .lineToLinearHeading(new Pose2d(-1, 33, Math.toRadians(180)))
                .back(8)
                .build();

        TrajectorySequence preloadBackboardLeftDelivery = driveTrain.trajectorySequenceBuilder(preloadDeliveryLeft.end())
                .back(10)
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(48, 38, Math.toRadians(180)))
                .back(7)
                .build();

        TrajectorySequence preloadBackboardMiddleDelivery = driveTrain.trajectorySequenceBuilder(preloadDeliveryMiddle.end())
                .back(7)
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(48, 31, Math.toRadians(180)))
                .back(8)
                .build();

        TrajectorySequence preloadBackboardRightDelivery = driveTrain.trajectorySequenceBuilder(preloadDeliveryRight.end())
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(48, 29, Math.toRadians(180)))
                .back(7)
                .build();

        TrajectorySequence parkLeft = driveTrain.trajectorySequenceBuilder(preloadBackboardLeftDelivery.end())
                .forward(15)
                .strafeRight(20)
                .back(15)
                .build();

        TrajectorySequence parkMiddle = driveTrain.trajectorySequenceBuilder(preloadBackboardMiddleDelivery.end())
                .forward(15)
                .strafeRight(25)
                .back(15)
                .build();

        TrajectorySequence parkRight = driveTrain.trajectorySequenceBuilder(preloadBackboardRightDelivery.end())
                .forward(15)
                .strafeRight(35)
                .back(15)
                .build();

        telemetry.addLine("Ready to start");
        telemetry.update();

        waitForStart();

        PropProcessor.Spikes spikePosition = processor.getSpikePosition();
        webcam.stopWebcam();

        telemetry.addData("Going to", spikePosition);
        telemetry.update();

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

        delivery.setDeliveryState(Delivery.DeliveryState.TRANSITION_1);
        intake.setServoPosition(Intake.IntakeState.STACK_HIGH);
        driveTrain.followTrajectorySequence(preloadDelivery);

        intake.reverse(0.5);
        timer.reset();
        while (timer.milliseconds() < 800) { }
        intake.off();

        delivery.setDeliveryState(Delivery.DeliveryState.TRANSITION_2);
        driveTrain.followTrajectorySequence(preloadDeliveryBackdrop);

        while (timer.milliseconds() < 250) {
        }

        delivery.setDeliveryState(Delivery.DeliveryState.SCORE_PRELOAD);
        timer.reset();
        while (timer.milliseconds() < 250) {
        }

        claw.setClawState(Claw.ClawState.OPEN_AUTO);

        timer.reset();
        while (timer.milliseconds() < 250) {
        }

        delivery.setDeliveryState(Delivery.DeliveryState.TRANSITION_1);
        driveTrain.followTrajectorySequence(park);
    }
}
