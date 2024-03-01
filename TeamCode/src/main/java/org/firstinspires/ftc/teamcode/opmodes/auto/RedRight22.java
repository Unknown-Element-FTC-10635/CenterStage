package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Blinkin;
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
import org.firstinspires.ftc.teamcode.vision.IntakeProcessor;
import org.firstinspires.ftc.teamcode.vision.PropProcessor;

@Autonomous(name = "RED (Backboard) - 2+2", group = "red")
public class RedRight22 extends OpMode {
    public enum AutoStates {
        START,
        SCORE_PURPLE_PRELOAD,
        SCORE_YELLOW_PRELOAD,
        DRIVE_TO_STACK,
        PICKUP_STACK_PIXELS,
        RETRY_STACK,
        SCORE_STACK_PIXELS,
        PARK,
        DONE,
        WAIT_ARRIVAL,
    }

    private SampleMecanumDrive driveTrain;
    private BreakBeam leftBeam, rightBeam;
    private LimitSwitch slideLimit;
    private Delivery delivery;
    private Blinkin blinkin;
    private Intake intake;
    private Slides slides;
    private Claw claw;

    private IntakeProcessor intakeProcessor;
    private PropProcessor processor;
    private Webcam webcam;

    private ElapsedTime timer;
    private ElapsedTime parkTimer;

    private TrajectorySequence  preloadDeliveryLeft, preloadDeliveryBackdropLeft, preloadDeliveryCenter,
                                preloadDeliveryBackdropCenter, preloadDeliveryRight, preloadDeliveryBackdropRight,
                                toCommonPathLeft, toCommonPathCenter, toCommonPathRight;
    private TrajectorySequence  preloadDelivery, preloadDeliveryBackdrop, toCommonPath, toStack, backToKnownPosition, stackDeliveryBackdrop, park;
    private Pose2d startPose;
    private AutoStates currentState, targetState;
    private boolean scored;
    private int subTransition;
    private int tries;

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
        blinkin = new Blinkin(hardwareMap);

        processor = new PropProcessor(false);
        webcam = new Webcam(hardwareMap, processor, "webcam");
        claw.setClawState(Claw.ClawState.SINGLE_CLOSED);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        startPose = new Pose2d(9, -62, Math.toRadians(90));
        driveTrain.setPoseEstimate(startPose);
        buildPaths();

        timer = new ElapsedTime();
        timer.startTime();
        parkTimer = new ElapsedTime();

        currentState = AutoStates.START;
    }

    @Override
    public void init_loop() {
        super.init_loop();

        telemetry.addData("Prop Location", processor.getSpikePosition());
        telemetry.update();
    }

    @Override
    public void start() {
        super.start();

        PropProcessor.Spikes spikePosition = processor.getSpikePosition();
        webcam.stopWebcam();

        intakeProcessor = new IntakeProcessor();
        webcam = new Webcam(hardwareMap, intakeProcessor, "intake webcam");

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
                delivery.setDeliveryState(Delivery.DeliveryState.TRANSITION_1);
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
                        delivery.setDeliveryState(Delivery.DeliveryState.TRANSITION_1);
                        driveTrain.followTrajectorySequenceAsync(preloadDeliveryBackdrop);

                        subTransition = 0;
                        targetState =  AutoStates.SCORE_YELLOW_PRELOAD;
                        currentState = AutoStates.WAIT_ARRIVAL;
                        break;
                }

                break;
            case SCORE_YELLOW_PRELOAD:
                switch (subTransition) {
                    case 0:
                        slides.setHeight(Slides.SlidesHeights.PRELOAD);

                        timer.reset();
                        subTransition++;
                        break;
                    case 1:
                        if (slides.atTargetPosition() || timerAt(250)) {
                            timer.reset();
                            subTransition++;
                        }

                        break;
                    case 2:
                        delivery.setDeliveryState(Delivery.DeliveryState.SCORE_PRELOAD);

                        timer.reset();
                        subTransition++;
                        break;
                    case 3:
                    case 7:
                        if (timerAt(250)) {
                            timer.reset();
                            subTransition++;
                        }

                        break;
                    case 4:
                        claw.setClawState(Claw.ClawState.OPEN_AUTO);

                        subTransition++;
                        timer.reset();
                        break;
                    case 5:
                        if (timerAt(100)) {
                            timer.reset();
                            subTransition++;
                        }

                        break;
                    case 6:
                        delivery.setDeliveryState(Delivery.DeliveryState.INTAKE_HOLD);

                        timer.reset();
                        subTransition++;
                        break;

                    case 8:
                        slides.setHeight(Slides.SlidesHeights.BASE);
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
                        claw.setClawState(Claw.ClawState.OPEN_INTAKE);
                        intake.on(0.8);
                        if (tries == 0) {
                            intake.setServoPosition(Intake.IntakeState.GROUND);
                        }

                        timer.reset();
                        subTransition++;
                        break;
                    case 1:
                        if (timerAt(550)) {
                            subTransition++;
                        }

                        break;
                    case 2:
                        driveTrain.setMotorPowers(0, 0, 0, 0);

                        subTransition++;
                        timer.reset();
                        break;
                    case 3:
                        if (timerAt(800)) {
                            subTransition++;
                        }

                        break;
                    case 4:
                        intake.on();
                        subTransition = 0;

                        if (intakeProcessor.hasTwoPixel()) {
                            driveTrain.followTrajectorySequenceAsync(backToKnownPosition);
                            targetState = AutoStates.SCORE_STACK_PIXELS;
                            currentState = AutoStates.WAIT_ARRIVAL;
                        } else if (tries < 2) {
                            intake.setServoPosition(Intake.IntakeState.GROUND);
                            currentState = AutoStates.RETRY_STACK;
                        } else {
                            driveTrain.followTrajectorySequenceAsync(backToKnownPosition);
                            targetState = AutoStates.PARK;
                            currentState = AutoStates.WAIT_ARRIVAL;
                        }
                        break;
                }
                break;
            case RETRY_STACK:
                switch (subTransition) {
                    case 0:
                        tries++;
                        intake.reverse(.5);
                        driveTrain.setMotorPowers(-0.65, 0.65, -0.65, 0.65);
                        intake.setServoPosition(Intake.IntakeState.GROUND);

                        timer.reset();
                        subTransition++;
                        break;
                    case 1:
                        if (timerAt(400)) {
                            subTransition++;
                        }

                        break;
                    case 2:
                        driveTrain.setMotorPowers(-0.2, -0.2, -0.2, -0.2);
                        intake.on();
                        timer.reset();
                        subTransition++;
                        break;
                    case 3:
                        if (timerAt(500)) {
                            subTransition++;
                        }

                        break;
                    case 4:
                        timer.reset();
                        subTransition = 0;
                        intake.setServoPosition(Intake.IntakeState.GROUND);
                        currentState = AutoStates.PICKUP_STACK_PIXELS;
                        break;
                }
                break;
            case SCORE_STACK_PIXELS:
                switch (subTransition) {
                    case 0:
                        delivery.setDeliveryState(Delivery.DeliveryState.INTAKE_PICKUP);
                        intake.setServoPosition(Intake.IntakeState.STACK_MID);
                        timer.reset();

                        subTransition++;
                        break;
                    case 1:
                        // Wait until we know the claw is parallel to the ground
                        if (timerAt(600)) {
                            subTransition++;
                        }

                        break;
                    case 2:
                        // Close claw onto the pixels
                        claw.setClawState(Claw.ClawState.CLOSED);
                        timer.reset();
                        intake.reverse();

                        subTransition++;
                        driveTrain.followTrajectorySequenceAsync(stackDeliveryBackdrop);
                        break;
                    case 3:
                        // Wait <milliseconds> so the physical servo has time to actually move
                        if (timerAt(725)) {
                            subTransition++;
                        }

                        break;
                    case 4:
                        intake.off();
                        // Move to safe transition point to avoid the cross-beam
                        delivery.setDeliveryState(Delivery.DeliveryState.INTAKE_HOLD);
                        intake.off();
                        timer.reset();

                        subTransition++;
                        break;
                    case 5:
                        if (timerAt(700)) {
                            subTransition++;
                        }

                        break;
                    case 6:
                        // Move to safe transition point that works for both intake and score
                        delivery.setDeliveryState(Delivery.DeliveryState.TRANSITION_1);
                        subTransition++;
                        timer.reset();
                        break;
                    case 7:
                        // Wait <milliseconds> so the physical servo has time to actually move
                        if (timerAt(700)) {
                            subTransition++;
                        }

                        break;
                    case 8:
                        // Move to safe transition point that works for both intake and score
                        intake.off();
                        delivery.setDeliveryState(Delivery.DeliveryState.TRANSITION_2);
                        subTransition++;
                        timer.reset();
                    case 9:
                        slides.setHeight(Slides.SlidesHeights.SECOND_LEVEL);
                        delivery.setDeliveryState(Delivery.DeliveryState.SCORE);

                        timer.reset();
                        subTransition++;
                        break;
                    case 10:
                        if (slides.atTargetPosition() || timerAt(500)) {
                            timer.reset();
                            subTransition++;
                        }

                        break;
                    case 11:
                        if (!driveTrain.isBusy()) {
                            timer.reset();
                            subTransition++;
                        }

                        break;
                    case 12:
                        if (timerAt(100)){
                            subTransition++;
                        }
                        break;
                    case 13:
                        claw.setClawState(Claw.ClawState.OPEN_SCORE);
                        timer.reset();

                        scored = true;
                        subTransition++;
                        break;
                    case 14:
                        if (timerAt(250)) {
                            subTransition++;
                        }

                        break;
                    case 15:
                        delivery.setDeliveryState(Delivery.DeliveryState.TRANSITION_2);
                        blinkin.clear();

                        timer.reset();
                        subTransition++;
                        break;
                    case 16:
                        if (timerAt(400)) {
                            subTransition++;
                        }

                        break;
                    case 17:
                        // Move to transition point between intake and score
                        slides.setHeight(Slides.SlidesHeights.BASE);
                        delivery.setDeliveryState(Delivery.DeliveryState.TRANSITION_1);

                        subTransition = 0;
                        currentState = AutoStates.DONE;
                        break;
                }

                break;
            case PARK:
                switch (subTransition) {
                    case 0:
                        parkTimer.startTime();
                        intake.off();
                        driveTrain.followTrajectorySequenceAsync(park);

                        subTransition++;
                        break;
                    case 1:
                        if (!driveTrain.isBusy()) {
                            subTransition++;
                        }

                        break;
                    case 2:
                        intake.reverse(1.0);
                        break;
                }
                break;
            case DONE:
                break;
            case WAIT_ARRIVAL:
                if (!driveTrain.isBusy()) {
                    currentState = targetState;
                }

                break;

        }

        if ((currentState == AutoStates.PICKUP_STACK_PIXELS || currentState == AutoStates.RETRY_STACK || (targetState == AutoStates.PARK && parkTimer.milliseconds() < 2000)) && intakeProcessor.hasTwoPixel()) {
            subTransition = 0;
            driveTrain.followTrajectorySequenceAsync(backToKnownPosition);
            targetState = AutoStates.SCORE_STACK_PIXELS;
            currentState = AutoStates.WAIT_ARRIVAL;
            intake.on();
        }

        if (slideLimit.isRisingEdge()) {
            slides.resetEncoders();
        }

        telemetry.addData("camera", intakeProcessor.hasTwoPixel());
        telemetry.update();
    }

    private void update() {
        driveTrain.update();

        delivery.update();
        slideLimit.update();
        slides.update();
        leftBeam.update();
        rightBeam.update();

        intakeProcessor.update();
        if (intakeProcessor.hasTwoPixel() || intakeProcessor.hasOnePixel()) {
            blinkin.setLEDColors(intakeProcessor.getLeftPixel(), intakeProcessor.getRightPixel());
        }
    }

    private boolean timerAt(double targetMS) {
        return timer.milliseconds() > targetMS;
    }

    private void buildPaths() {
        preloadDeliveryLeft = driveTrain.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(15, -52))
                .lineToLinearHeading(new Pose2d(2, -31, Math.toRadians(180)))
                .build();

        preloadDeliveryCenter = driveTrain.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(15, -31))
                .build();

        preloadDeliveryRight = driveTrain.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(14, -53))
                .splineTo(new Vector2d(29,-36), Math.toRadians(140))
                .build();

        preloadDeliveryBackdropLeft = driveTrain.trajectorySequenceBuilder(preloadDeliveryLeft.end())
                .setReversed(true)
                .splineTo(new Vector2d(50, -28), Math.toRadians(0))
                .back(3)
                .build();

        preloadDeliveryBackdropCenter = driveTrain.trajectorySequenceBuilder(preloadDeliveryCenter.end())
                .back(7)
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(46, -35, Math.toRadians(180)))
                .back(6)
                .build();

        preloadDeliveryBackdropRight = driveTrain.trajectorySequenceBuilder(preloadDeliveryRight.end())
                .setReversed(true)
                .splineTo(new Vector2d(57, -40), Math.toRadians(0))
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
                .splineTo(new Vector2d(20, -10), Math.toRadians(180))
                .splineTo(new Vector2d(-60, -5), Math.toRadians(195))
                .build();

        backToKnownPosition = driveTrain.trajectorySequenceBuilder(new Pose2d(-60, 9, Math.toRadians(180)))
                .setReversed(true)
                .lineTo(new Vector2d(-45, -7))
                .build();

        stackDeliveryBackdrop = driveTrain.trajectorySequenceBuilder(backToKnownPosition.end())
                .setReversed(true)
                .lineTo(new Vector2d(10, -5))
                .splineTo(new Vector2d(44, -36), Math.toRadians(0))
                .back(6)
                .build();

        park = driveTrain.trajectorySequenceBuilder(backToKnownPosition.end())
                .setReversed(true)
                .lineTo(new Vector2d(10, -5))
                .splineTo(new Vector2d(50, -14), Math.toRadians(0))
                .turn(Math.toRadians(180))
                .build();
    }
}
