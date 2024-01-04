package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.BreakBeam;
import org.firstinspires.ftc.teamcode.hardware.Claw;
import org.firstinspires.ftc.teamcode.hardware.Delivery;
import org.firstinspires.ftc.teamcode.hardware.Intake;
import org.firstinspires.ftc.teamcode.hardware.LimitSwitch;
import org.firstinspires.ftc.teamcode.hardware.Slides;
import org.firstinspires.ftc.teamcode.hardware.Webcam;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.utils.CurrentOpmode;
import org.firstinspires.ftc.teamcode.vision.PropProcessor;

@Autonomous(name = "BLUE (Left) - 2+2")
public class BlueLeft22 extends OpMode {
    public enum AutoStates {
        START,
        SCORE_PURPLE_PRELOAD,
        SCORE_YELLOW_PRELOAD,
        DRIVE_TO_STACK,
        PICKUP_STACK_PIXELS,
        SCORE_STACK_PIXELS,
        PARK,
        WAIT_ARRIVAL,
    }

    private SampleMecanumDrive driveTrain;
    private BreakBeam leftBeam, rightBeam;
    private LimitSwitch slideLimit;
    private Delivery delivery;
    private Intake intake;
    private Slides slides;
    private Claw claw;

    private Webcam webcam;

    private ElapsedTime timer;

    private TrajectorySequence  preloadDeliveryLeft, preloadDeliveryBackdropLeft, preloadDeliveryCenter,
                                preloadDeliveryBackdropCenter, preloadDeliveryRight, preloadDeliveryBackdropRight,
                                toCommonPathLeft, toCommonPathCenter, toCommonPathRight;
    private TrajectorySequence  preloadDelivery, preloadDeliveryBackdrop, toCommonPath, toStack, backToKnownPosition, stackDeliveryBackdrop, park;
    private Pose2d startPose;
    private AutoStates currentState, targetState;
    private int subTransition;

    @Override
    public void init() {
        CurrentOpmode.setCurrentOpmode(CurrentOpmode.OpMode.AUTO);

        slideLimit = new LimitSwitch(hardwareMap, "slide limit");
        rightBeam = new BreakBeam(hardwareMap, "right break");
        leftBeam = new BreakBeam(hardwareMap, "left break");
        driveTrain = new SampleMecanumDrive(hardwareMap);
        delivery = new Delivery(hardwareMap);
        intake = new Intake(hardwareMap);
        slides = new Slides(hardwareMap);
        claw = new Claw(hardwareMap);

        webcam = new Webcam(hardwareMap, true);
        claw.setClawState(Claw.ClawState.SINGLE_CLOSED);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        startPose = new Pose2d( 9, 62, Math.toRadians(270));
        driveTrain.setPoseEstimate(startPose);
        buildPaths();

        timer = new ElapsedTime();
        timer.startTime();

        currentState = AutoStates.START;
    }

    @Override
    public void init_loop() {
        super.init_loop();

        telemetry.addData("Prop Location", webcam.propProcessor.getSpikePosition());
        telemetry.update();
    }

    @Override
    public void start() {
        super.start();

        PropProcessor.Spikes spikePosition = webcam.propProcessor.getSpikePosition();
        webcam.stopWebcam();

        switch (spikePosition) {
            case LEFT:
                preloadDelivery = preloadDeliveryLeft;
                preloadDeliveryBackdrop = preloadDeliveryBackdropLeft;
                toCommonPath = toCommonPathLeft;
                break;
            case CENTER:
                preloadDelivery = preloadDeliveryCenter;
                preloadDeliveryBackdrop = preloadDeliveryBackdropCenter;
                toCommonPath = toCommonPathCenter;
                break;
            case RIGHT:
                preloadDelivery = preloadDeliveryRight;
                preloadDeliveryBackdrop = preloadDeliveryBackdropRight;
                toCommonPath = toCommonPathRight;
                break;
        }
    }

    @Override
    public void loop() {
        update();

        switch (currentState) {
            case START:
                delivery.setDeliveryState(Delivery.DeliveryState.INTAKE_HOLD);
                intake.setServoPosition(Intake.IntakeState.STACK_HIGH);
                driveTrain.followTrajectorySequenceAsync(preloadDelivery);

                targetState = AutoStates.SCORE_PURPLE_PRELOAD;
                currentState = AutoStates.WAIT_ARRIVAL;
                break;
            case SCORE_PURPLE_PRELOAD:
                switch (subTransition) {
                    case 0:
                        intake.reverse(0.5);
                        timer.reset();

                        subTransition++;
                        break;
                    case 1:
                        if (timerAt(800)) {
                            subTransition++;
                        }

                        break;
                    case 2:
                        intake.off();
                        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                        delivery.setDeliveryState(Delivery.DeliveryState.TRANSITION_2);
                        driveTrain.followTrajectorySequenceAsync(preloadDeliveryBackdrop);

                        subTransition = 0;
                        targetState = AutoStates.SCORE_YELLOW_PRELOAD;
                        currentState = AutoStates.WAIT_ARRIVAL;
                        break;
                }

                break;
            case SCORE_YELLOW_PRELOAD:
                switch (subTransition) {
                    case 0:
                        slides.setHeight(Slides.SlidesHeights.AUTO);
                        delivery.setDeliveryState(Delivery.DeliveryState.SCORE_AUTO);

                        timer.reset();
                        subTransition++;
                        break;
                    case 1:
                        if (slides.atTargetPosition() || timerAt(700)) {
                            timer.reset();
                            subTransition++;
                        }

                        break;
                    case 2:
                        if (timerAt(250)) {
                            subTransition++;
                        }

                        break;
                    case 3:
                        claw.setClawState(Claw.ClawState.OPEN_INTAKE);
                        claw.depowerServo();

                        timer.reset();
                        subTransition++;
                        break;
                    case 4:
                        if (timerAt(1000)) {
                            subTransition++;
                        }

                        break;
                    case 5:
                        delivery.setDeliveryState(Delivery.DeliveryState.TRANSITION_2);

                        subTransition++;
                        break;
                    case 6:
                        if (timerAt(500)) {
                            subTransition++;
                        }

                        break;
                    case 7:
                        slides.setHeight(Slides.SlidesHeights.BASE);
                        intake.setServoPosition(Intake.IntakeState.STACK_HIGH);
                        driveTrain.followTrajectorySequenceAsync(toCommonPath);

                        subTransition = 0;
                        targetState = AutoStates.DRIVE_TO_STACK;
                        currentState = AutoStates.WAIT_ARRIVAL;
                        break;
                }

                break;
            case DRIVE_TO_STACK:
                driveTrain.followTrajectorySequence(toStack);

                targetState = AutoStates.PICKUP_STACK_PIXELS;
                currentState = AutoStates.WAIT_ARRIVAL;
                break;
            case PICKUP_STACK_PIXELS:
                switch (subTransition) {
                    case 0:
                        driveTrain.setMotorPowers(0.25, 0.25, 0.25, 0.25);
                        delivery.setDeliveryState(Delivery.DeliveryState.INTAKE_HOLD);
                        claw.repowerServo();
                        claw.setClawState(Claw.ClawState.OPEN_INTAKE);
                        intake.on(0.8);
                        intake.setServoPosition(Intake.IntakeState.STACK_HIGH);

                        timer.reset();
                        subTransition++;
                        break;
                    case 1:
                        if (timer.milliseconds() > 500) {
                            subTransition++;
                        }

                        break;
                    case 2:
                        driveTrain.setMotorPowers(0, 0, 0, 0);

                        subTransition++;
                        timer.reset();
                        break;
                    case 3:
                        if (timer.milliseconds() > 1500) {
                            subTransition++;
                        }

                        break;
                    case 4:
                        driveTrain.followTrajectorySequenceAsync(backToKnownPosition);
                        intake.on();

                        subTransition = 0;
                        targetState = AutoStates.SCORE_STACK_PIXELS;
                        currentState = AutoStates.WAIT_ARRIVAL;
                        break;
                }
                break;
            case SCORE_STACK_PIXELS:
                switch (subTransition) {
                    case 0:
                        driveTrain.followTrajectorySequenceAsync(stackDeliveryBackdrop);
                        delivery.setDeliveryState(Delivery.DeliveryState.INTAKE_PICKUP);
                        timer.reset();

                        subTransition++;
                        break;
                    case 1:
                    case 3:
                    case 11:
                    case 13:
                        // Wait <milliseconds> so the physical servo has time to actually move
                        // Wait until we know the claw is parallel to the ground
                        if (timer.milliseconds() > 500) {
                            subTransition++;
                        }

                        break;
                    case 2:
                        // Close claw onto the pixels
                        claw.setClawState(Claw.ClawState.CLOSED);
                        timer.reset();

                        subTransition++;
                        break;

                    case 4:
                        // Move to safe transition point to avoid the cross-beam
                        delivery.setDeliveryState(Delivery.DeliveryState.INTAKE_HOLD);
                        intake.off();
                        timer.reset();

                        subTransition++;
                        break;
                    case 5:
                        if (timer.milliseconds() > 1250) {
                            subTransition++;
                        }

                        break;
                    case 6:
                        // Move to safe transition point that works for both intake and score
                        delivery.setDeliveryState(Delivery.DeliveryState.TRANSITION_2);
                        subTransition++;
                        timer.reset();
                        break;
                    case 7:
                        if (!driveTrain.isBusy()) {
                            subTransition++;
                        }

                        break;
                    case 8:
                        slides.setHeight(Slides.SlidesHeights.SECOND_LEVEL);
                        delivery.setDeliveryState(Delivery.DeliveryState.SCORE_AUTO);

                        timer.reset();
                        subTransition++;
                        break;
                    case 9:
                        if (slides.atTargetPosition() || timerAt(1000)) {
                            timer.reset();
                            subTransition++;
                        }

                        break;
                    case 10:
                        claw.setClawState(Claw.ClawState.OPEN_SCORE);

                        timer.reset();
                        subTransition++;
                        break;
                    case 12:
                        delivery.setDeliveryState(Delivery.DeliveryState.TRANSITION_2);

                        subTransition++;
                    case 14:
                        slides.setHeight(Slides.SlidesHeights.BASE);

                        subTransition = 0;
                        currentState = AutoStates.PARK;
                        break;
                }

                break;
            case PARK:
                break;
            case WAIT_ARRIVAL:
                if (!driveTrain.isBusy()) {
                    currentState = targetState;
                }

                break;
        }

        if (slideLimit.isRisingEdge()) {
            slides.resetEncoders();
        }
    }

    private void update() {
        driveTrain.update();

        delivery.update();
        slideLimit.update();
        slides.update();
        leftBeam.update();
        rightBeam.update();
    }

    private boolean timerAt(double targetMS) {
        return timer.milliseconds() > targetMS;
    }

    private void buildPaths() {
        preloadDeliveryLeft = driveTrain.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(14, 53))
                .lineToLinearHeading(new Pose2d(12, 32, Math.toRadians(180)))
                .build();

        preloadDeliveryCenter = driveTrain.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(14, 30))
                .build();

        preloadDeliveryRight = driveTrain.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(14, 53))
                .splineTo(new Vector2d(25, 38.5), Math.toRadians(230))
                .build();

        preloadDeliveryBackdropLeft = driveTrain.trajectorySequenceBuilder(preloadDeliveryLeft.end())
                .setReversed(true)
                .splineTo(new Vector2d(50, 41.5), Math.toRadians(0))
                .build();

        preloadDeliveryBackdropCenter = driveTrain.trajectorySequenceBuilder(preloadDeliveryCenter.end())
                .back(6)
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(50, 32, Math.toRadians(180)))
                .back(7)
                .build();

        preloadDeliveryBackdropRight = driveTrain.trajectorySequenceBuilder(preloadDeliveryRight.end())
                .setReversed(true)
                .splineTo(new Vector2d(50, 30), Math.toRadians(0))
                .build();

        toCommonPathLeft = driveTrain.trajectorySequenceBuilder(preloadDeliveryBackdropLeft.end())
                .forward(5)
                .build();

        toCommonPathCenter = driveTrain.trajectorySequenceBuilder(preloadDeliveryBackdropCenter.end())
                .forward(7)
                .build();

        toCommonPathRight = driveTrain.trajectorySequenceBuilder(preloadDeliveryBackdropRight.end())
                .forward(5)
                .build();

        toStack = driveTrain.trajectorySequenceBuilder(toCommonPathCenter.end())
                .splineTo(new Vector2d(20, 5), Math.toRadians(180))
                .splineTo(new Vector2d(-59, 8), Math.toRadians(180))
                .build();

        backToKnownPosition = driveTrain.trajectorySequenceBuilder(new Pose2d(-60, 9, Math.toRadians(180)))
                .setReversed(true)
                .lineTo(new Vector2d(-45, 7))
                .build();

        stackDeliveryBackdrop = driveTrain.trajectorySequenceBuilder(backToKnownPosition.end())
                .setReversed(true)
                .lineTo(new Vector2d(25, 7))
                .splineTo(new Vector2d(46, 32), Math.toRadians(0))
                .back(7)
                .build();
    }
}
