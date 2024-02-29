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
import org.firstinspires.ftc.teamcode.utils.PixelColors;
import org.firstinspires.ftc.teamcode.vision.IntakeProcessor;
import org.firstinspires.ftc.teamcode.vision.PropProcessor;

@Autonomous(name = "BLUE (Backboard) - 2+2", group = "blue")
public class BlueLeft22 extends OpMode {
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

    private PropProcessor processor;
    private IntakeProcessor intakeProcessor;
    private Webcam webcam;

    private ElapsedTime timer;
    private ElapsedTime parkTimer;

    private TrajectorySequence  preloadDeliveryLeft, preloadDeliveryBackdropLeft, preloadDeliveryCenter,
                                preloadDeliveryBackdropCenter, preloadDeliveryRight, preloadDeliveryBackdropRight,
                                toCommonPathLeft, toCommonPathCenter, toCommonPathRight;
    private TrajectorySequence  preloadDelivery, preloadDeliveryBackdrop, toCommonPath, toStack, backToKnownPosition, stackDeliveryBackdrop, park;
    private Pose2d startPose;
    private AutoStates currentState, targetState;
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
        blinkin = new Blinkin(hardwareMap);
        intake = new Intake(hardwareMap);
        slides = new Slides(hardwareMap);
        claw = new Claw(hardwareMap);

        processor = new PropProcessor(true);
        webcam = new Webcam(hardwareMap, processor, "webcam");
        claw.setClawState(Claw.ClawState.SINGLE_CLOSED);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        startPose = new Pose2d( 9, 62, Math.toRadians(270));
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
                        delivery.setDeliveryState(Delivery.DeliveryState.SCORE);

                        timer.reset();
                        subTransition++;
                        break;
                    case 1:
                    case 3:
                        if (timerAt(250)) {
                            timer.reset();
                            subTransition++;
                        }

                        break;
                    case 2:
                        claw.setClawState(Claw.ClawState.OPEN_AUTO);

                        subTransition++;
                        timer.reset();
                        break;
                    case 4:
                        delivery.setDeliveryState(Delivery.DeliveryState.TRANSITION_1);
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
                        intake.setServoPosition(Intake.IntakeState.STACK_HIGH);

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
                        intake.reverse(0.25);
                        driveTrain.setMotorPowers(0.75, -0.75, 0.75, -0.75);

                        timer.reset();
                        subTransition++;
                        break;
                    case 1:
                    case 3:
                        if (timerAt(300)) {
                            subTransition++;
                        }

                        break;
                    case 2:
                        driveTrain.setMotorPowers(-0.2, -0.2, -0.2, -0.2);
                        intake.on();
                        timer.reset();
                        subTransition++;
                        break;
                    case 4:
                        timer.reset();
                        subTransition = 0;
                        if (tries == 1) {
                            intake.setServoPosition(Intake.IntakeState.STACK_AUTO);
                        } else if (tries == 2) {
                            intake.setServoPosition(Intake.IntakeState.STACK_MID);
                        }
                        currentState = AutoStates.PICKUP_STACK_PIXELS;
                        break;
                }
                break;
            case SCORE_STACK_PIXELS:
                switch (subTransition) {
                    case 0:
                        driveTrain.followTrajectorySequenceAsync(stackDeliveryBackdrop);
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

                        subTransition++;
                        break;
                    case 3:
                        // Wait <milliseconds> so the physical servo has time to actually move
                        if (timerAt(725)) {
                            subTransition++;
                        }

                        break;
                    case 4:
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
                        if (timerAt(500)) {
                            subTransition++;
                        }

                        break;
                    case 8:
                        // Move to safe transition point that works for both intake and score
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
                            subTransition++;
                        }

                        break;
                    case 12:
                        claw.setClawState(Claw.ClawState.OPEN_SCORE);
                        timer.reset();

                        subTransition++;
                        break;
                    case 13:
                        if (timerAt(250)) {
                            subTransition++;
                        }

                        break;
                    case 14:
                        delivery.setDeliveryState(Delivery.DeliveryState.TRANSITION_2);
                        blinkin.clear();

                        timer.reset();
                        subTransition++;
                        break;
                    case 15:
                        if (timerAt(400)) {
                            subTransition++;
                        }

                        break;
                    case 16:
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
                        intake.reverse();
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

        intakeProcessor.update();
        if (intakeProcessor.hasTwoPixel() || intakeProcessor.hasOnePixel()) {
            blinkin.setLEDColors(intakeProcessor.getLeftPixel(), intakeProcessor.getRightColor());
        }
    }

    private boolean timerAt(double targetMS) {
        return timer.milliseconds() > targetMS;
    }

    private void buildPaths() {
        preloadDeliveryLeft = driveTrain.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(14, 53))
                .splineTo(new Vector2d(27,36), Math.toRadians(230))
                .build();

        preloadDeliveryCenter = driveTrain.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(14, 30))
                .build();

        preloadDeliveryRight = driveTrain.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(15, 52))
                .lineToLinearHeading(new Pose2d(0, 28, Math.toRadians(180)))
                .build();

        preloadDeliveryBackdropLeft = driveTrain.trajectorySequenceBuilder(preloadDeliveryLeft.end())
                .setReversed(true)
                .splineTo(new Vector2d(52, 38), Math.toRadians(0))
                .back(3)
                .build();

        preloadDeliveryBackdropCenter = driveTrain.trajectorySequenceBuilder(preloadDeliveryCenter.end())
                .back(6)
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(48, 28, Math.toRadians(180)))
                .back(7)
                .build();

        preloadDeliveryBackdropRight = driveTrain.trajectorySequenceBuilder(preloadDeliveryRight.end())
                .setReversed(true)
                .splineTo(new Vector2d(49, 25), Math.toRadians(0))
                .back(2)
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
                .splineTo(new Vector2d(-61, 11), Math.toRadians(180))
                .build();

        backToKnownPosition = driveTrain.trajectorySequenceBuilder(new Pose2d(-60, 9, Math.toRadians(180)))
                .setReversed(true)
                .lineTo(new Vector2d(-45, 7))
                .build();

        stackDeliveryBackdrop = driveTrain.trajectorySequenceBuilder(backToKnownPosition.end())
                .setReversed(true)
                .lineTo(new Vector2d(22, 7))
                .splineTo(new Vector2d(45, 29), Math.toRadians(0))
                .back(2)
                .build();

        park = driveTrain.trajectorySequenceBuilder(backToKnownPosition.end())
                .setReversed(true)
                .splineTo(new Vector2d(45, 7), Math.toRadians(0))
                .turn(-Math.toRadians(180))
                .build();
    }
}
