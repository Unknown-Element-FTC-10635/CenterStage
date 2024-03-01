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
import org.firstinspires.ftc.teamcode.hardware.StackColorSensor;
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
import org.opencv.core.Mat;

@Autonomous(name = "BLUE (Backboard) - 2+2", group = "blue")
public class BlueLeft22 extends OpMode {
    public enum AutoStates {
        START,
        SCORE_PURPLE_PRELOAD,
        SCORE_YELLOW_PRELOAD,
        DRIVE_TO_STACK,
        ALIGN_TO_STACK,
        PICKUP_STACK_PIXELS,
        RETRY_STACK,
        DRIVE_THROUGH_BARRIER,
        SCORE_STACK_PIXELS,
        PARK,
        DONE,
        WAIT_ARRIVAL,
        IS_STALLED
    }

    private SampleMecanumDrive driveTrain;
    private BreakBeam leftBeam, rightBeam;
    private StackColorSensor colorSensor;
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
            toCommonPathLeft, toCommonPathCenter, toCommonPathRight, toStackRight, toStackLeft, toStackCenter, backToKnownPositionRight, stackDeliveryBackdropRight;
    private TrajectorySequence  preloadDelivery, preloadDeliveryBackdrop, toCommonPath, toStack, backToKnownPosition, stackDeliveryBackdrop, park, park2;
    private Pose2d startPose;
    private AutoStates currentState, targetState;
    private int subTransition, saveTransition;
    private int tries;
    private boolean tryToScore;
    private int numOfStalls;
    @Override
    public void init() {
        CurrentOpmode.setCurrentOpmode(CurrentOpmode.OpMode.AUTO);

        colorSensor = new StackColorSensor(hardwareMap, "left color");
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

        startPose = new Pose2d( 9, 61, Math.toRadians(270));
        driveTrain.setPoseEstimate(startPose);
        buildPaths();

        timer = new ElapsedTime();
        timer.startTime();

        parkTimer = new ElapsedTime();

        currentState = AutoStates.START;
        numOfStalls = 0;
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
                toStack = toStackCenter;

                break;
            case CENTER:
                preloadDelivery = preloadDeliveryCenter;
                preloadDeliveryBackdrop = preloadDeliveryBackdropCenter;
                toCommonPath = toCommonPathCenter;
                toStack = toStackCenter;
                break;
            case RIGHT:
                preloadDelivery = preloadDeliveryRight;
                preloadDeliveryBackdrop = preloadDeliveryBackdropRight;
                toCommonPath = toCommonPathRight;
                toStack = toStackRight;
                backToKnownPosition = backToKnownPositionRight;
                stackDeliveryBackdrop = stackDeliveryBackdropRight;
                break;
        }
    }

    @Override
    public void loop() {
        update();

        switch (currentState) {
            case START:


                switch(subTransition){
                    case 0:

                        delivery.setDeliveryState(Delivery.DeliveryState.TRANSITION_1);
                        intake.setServoPosition(Intake.IntakeState.STACK_HIGH);
                        driveTrain.followTrajectorySequenceAsync(preloadDeliveryBackdrop);
                        timer.reset();
                        subTransition++;
                        break;
                    case 1:
                        if (timerAt(1000)) {
                            timer.reset();
                            subTransition++;
                        }
                        break;
                    case 2:
                        slides.setHeight(Slides.SlidesHeights.PRELOAD);
                        delivery.setDeliveryState(Delivery.DeliveryState.SCORE_PRELOAD);
                        subTransition = 0;

                        targetState = AutoStates.SCORE_YELLOW_PRELOAD;
                        currentState = AutoStates.WAIT_ARRIVAL;
                        break;
                }
                break;
            case SCORE_YELLOW_PRELOAD:
                switch (subTransition) {
                    case 0:
                        timer.reset();
                        subTransition++;
                        break;
                    case 1:
                        claw.setClawState(Claw.ClawState.OPEN_AUTO);

                        subTransition++;
                        timer.reset();
                        break;
                    case 2:
                        if (timerAt(500)) {
                            timer.reset();
                            subTransition++;
                        }

                        break;
                    case 3:
                        delivery.setDeliveryState(Delivery.DeliveryState.TRANSITION_2);
                        intake.setServoPosition(Intake.IntakeState.STACK_HIGH);
                        driveTrain.followTrajectorySequenceAsync(preloadDelivery);
                        slides.setHeight(Slides.SlidesHeights.BASE);

                        subTransition = 0;
                        targetState = AutoStates.SCORE_PURPLE_PRELOAD;
                        currentState = AutoStates.WAIT_ARRIVAL;
                        break;
                }

                break;
            case SCORE_PURPLE_PRELOAD:
                switch (subTransition) {
                    case 0:
                        intake.reverse(0.3);
                        timer.reset();

                        subTransition++;
                        break;
                    case 1:
                        if (timerAt(800)) {
                            subTransition++;
                        }

                        break;
                    case 2:
                        driveTrain.followTrajectorySequence(toCommonPath);
                        intake.off();
                        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                        intake.setServoPosition(Intake.IntakeState.GROUND);
                        delivery.setDeliveryState(Delivery.DeliveryState.INTAKE_HOLD);

                        subTransition = 0;
                        targetState = AutoStates.DRIVE_TO_STACK;
                        currentState = AutoStates.WAIT_ARRIVAL;
                        break;
                }

                break;

            case DRIVE_TO_STACK:
                switch (subTransition) {
                    case 0:
                        intake.off();
                        driveTrain.turn(Math.toRadians(180) - (driveTrain.getPoseEstimate().getHeading() * 1.2));
                        subTransition++;
                        break;
                    case 1:
                        if (!driveTrain.isBusy()) {
                            subTransition++;
                        }

                        break;
                    case 2:
                        intake.on();
                        driveTrain.followTrajectorySequence(toStack);

                        subTransition = 0;
                        targetState = AutoStates.ALIGN_TO_STACK;
                        currentState = AutoStates.WAIT_ARRIVAL;
                        break;
                }
                break;
            case ALIGN_TO_STACK:
                switch (subTransition)  {
                    case 0:
                        intake.setServoPosition(Intake.IntakeState.GROUND);
                        driveTrain.setMotorPowers(.5, -.5, .5, -.5);
                        timer.reset();
                        subTransition++;
                        break;
                    case 1:
                        if (colorSensor.linedUpWithStack() || timerAt(2000)) {
                            subTransition++;
                        }
                        break;
                    case 2:
                        driveTrain.setMotorPowers(0.25, 0.25, 0.25, 0.25);
                        timer.reset();
                        subTransition++;
                    case 3:
                        if (colorSensor.correctDistanceFromStack() || timerAt(500)) {
                            subTransition++;
                        }
                        break;
                    case 4:
                        subTransition = 0;
                        if (!intake.isStalled()) {
                            driveTrain.followTrajectorySequenceAsync(driveTrain.trajectorySequenceBuilder(driveTrain.getPoseEstimate())
                                    .strafeRight(8)
                                    .build());
                        }
                        targetState = AutoStates.PICKUP_STACK_PIXELS;
                        currentState = AutoStates.WAIT_ARRIVAL;
                        break;
                }
                break;
            case PICKUP_STACK_PIXELS:
                switch (subTransition) {
                    case 0:
                        delivery.setDeliveryState(Delivery.DeliveryState.INTAKE_HOLD);
                        claw.setClawState(Claw.ClawState.OPEN_INTAKE);
                        intake.on(0.8);
                        timer.reset();
                        subTransition++;
                        break;
                    case 1:
                        if (!driveTrain.isBusy()) {
                            subTransition++;
                        }
                        break;
                    case 2:
                        if (tries == 0) {
                            if (timerAt(1100)) {
                                subTransition++;
                            }
                        } else {
                            if (timerAt(1650)) {
                                subTransition++;
                            }
                        }
                        break;
                    case 3:
                        intake.on();

                        subTransition++;
                        timer.reset();
                        break;
                    case 4:
                        if (timerAt(500)) {
                            subTransition++;
                        }

                        break;
                    case 5:
                        subTransition = 0;

                        if (tries < 1) {
                            currentState = AutoStates.RETRY_STACK;
                        } else {
                            tryToScore = intakeProcessor.hasTwoPixel();
                            currentState = AutoStates.DRIVE_THROUGH_BARRIER;
                        }
                        break;
                }
                break;
            case RETRY_STACK:
                switch (subTransition) {
                    case 0:
                    case 4:
                        tries++;
                        intake.setServoPosition(Intake.IntakeState.STACK_MID);
                        intake.reverse(.35);
                        driveTrain.setMotorPowers(-.4, -.3, .4, .3);

                        timer.reset();
                        subTransition++;
                        break;
                    case 1:
                    case 7:
                        if (timerAt(100)) {
                            subTransition++;
                        }

                        break;
                    case 2:
                    case 6:
                        driveTrain.setMotorPowers(.3,.4,-.3,-.4);
                        intake.setServoPosition(Intake.IntakeState.GROUND);
                        intake.on();
                        timer.reset();
                        subTransition++;
                        break;
                    case 3:
                    case 5:
                        if (timerAt(200)) {
                            subTransition++;
                        }

                        break;
                    case 8:
                        driveTrain.setMotorPowers(0, 0, 0, 0);
                        timer.reset();
                        subTransition = 0;
                        if (tries == 1) {
                            intake.setServoPosition(Intake.IntakeState.GROUND);
                        }
                        currentState = AutoStates.DRIVE_THROUGH_BARRIER;
                        break;
                }
                break;
            case DRIVE_THROUGH_BARRIER:
                switch (subTransition) {
                    case 0:
                        intake.setServoPosition(Intake.IntakeState.STACK_HIGH);
                        intake.off();
                        timer.reset();

                        subTransition++;
                        break;
                    case 1:
                        if (timerAt(200)){
                            subTransition++;
                        }
                        break;
                    case 2:
                        driveTrain.followTrajectorySequenceAsync(backToKnownPosition);
                        intake.setServoPosition(Intake.IntakeState.STACK_MID);
                        timer.reset();
                        intake.reverse();

                        subTransition++;
                        break;

                    case 3:
                        if (timerAt(199)){
                            subTransition++;
                            intake.on();
                            timer.reset();
                        }
                        break;
                    case 4:
                        if (timerAt(200)){
                            subTransition++;
                        }
                    case 5:
                        delivery.setDeliveryState(Delivery.DeliveryState.INTAKE_PICKUP);
                        timer.reset();
                        subTransition++;
                        break;
                    case 6:
                        // Wait until we know the claw is parallel to the ground
                        if (timerAt(600)) {
                            subTransition++;
                        }

                        break;

                    case 7:
                        // Close claw onto the pixels
                        claw.setClawState(Claw.ClawState.CLOSED);
                        timer.reset();


                        subTransition++;
                        break;
                    case 8:
                        // Wait <milliseconds> so the physical servo has time to actually move
                        if (timerAt(725)) {
                            subTransition++;
                        }

                        break;
                    case 9:
                        // Move to safe transition point to avoid the cross-beam
                        delivery.setDeliveryState(Delivery.DeliveryState.INTAKE_HOLD);
                        intake.off();
                        timer.reset();

                        subTransition++;
                        break;
                    case 10:
                        if (!driveTrain.isBusy()) {
                            subTransition = 0;
                            if (tryToScore) {
                                currentState = AutoStates.SCORE_STACK_PIXELS;
                            } else{
                                currentState = AutoStates.PARK;
                            }
                        }
                        break;
                }
                break;
            case SCORE_STACK_PIXELS:
                switch(subTransition){
                    case 0:
                        driveTrain.followTrajectorySequenceAsync(stackDeliveryBackdrop);
                    case 1:
                        if (timerAt(700)) {
                            subTransition++;
                        }

                        break;
                    case 2:
                        // Move to safe transition point that works for both intake and score
                        delivery.setDeliveryState(Delivery.DeliveryState.TRANSITION_1);
                        subTransition++;
                        timer.reset();
                        break;
                    case 3:
                        // Wait <milliseconds> so the physical servo has time to actually move
                        if (timerAt(500)) {
                            subTransition++;
                        }

                        break;
                    case 4:
                        // Move to safe transition point that works for both intake and score
                        delivery.setDeliveryState(Delivery.DeliveryState.TRANSITION_2);
                        subTransition++;
                        timer.reset();
                    case 5:
                        slides.setHeight(Slides.SlidesHeights.FOURTH_LEVEL);
                        delivery.setDeliveryState(Delivery.DeliveryState.SCORE);

                        timer.reset();
                        subTransition++;
                        break;
                    case 6:
                        if (slides.atTargetPosition() || timerAt(500)) {
                            timer.reset();
                            subTransition++;
                        }

                        break;
                    case 7:
                        if (!driveTrain.isBusy()) {
                            subTransition++;
                        }

                        break;
                    case 8:
                        claw.setClawState(Claw.ClawState.OPEN_SCORE);
                        timer.reset();

                        subTransition++;
                        break;
                    case 9:
                        if (timerAt(250)) {
                            subTransition++;
                        }

                        break;
                    case 10:
                        delivery.setDeliveryState(Delivery.DeliveryState.TRANSITION_2);
                        blinkin.clear();

                        timer.reset();
                        subTransition++;
                        break;
                    case 11:
                        if (timerAt(400)) {
                            subTransition++;
                        }

                        break;
                    case 12:
                        // Move to transition point between intake and score
                        slides.setHeight(Slides.SlidesHeights.BASE);
                        delivery.setDeliveryState(Delivery.DeliveryState.TRANSITION_1);
                        driveTrain.followTrajectorySequenceAsync(park2);

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
            case IS_STALLED:
                switch (subTransition){
                    case 0:
                        driveTrain.setMotorPowers(-.2, -.2, -.2, -.2);
                        intake.reverse();
                        intake.setServoPosition(Intake.IntakeState.STACK_HIGH);
                        timer.reset();

                        subTransition++;
                        break;
                    case 1:
                        if (timerAt(300)){
                            intake.on();
                            intake.setServoPosition(Intake.IntakeState.GROUND);
                            subTransition++;
                            driveTrain.setMotorPowers(0,0,0,0);
                        }
                        break;
                    case 2:
                        subTransition = saveTransition;
                        currentState = targetState;
                        numOfStalls++;
                        timer.reset();
                        break;
                }
                break;
        }

        if ((currentState == AutoStates.PICKUP_STACK_PIXELS || currentState == AutoStates.RETRY_STACK || currentState == AutoStates.IS_STALLED || currentState == AutoStates.ALIGN_TO_STACK) && intakeProcessor.hasTwoPixel()) {
            tryToScore = true;
            subTransition = 0;
            currentState = AutoStates.DRIVE_THROUGH_BARRIER;
        } else if (currentState == AutoStates.DRIVE_THROUGH_BARRIER && intakeProcessor.hasTwoPixel()){
            tryToScore = true;
        }

        if (slideLimit.isRisingEdge()) {
            slides.resetEncoders();
        }

        if ((intake.isStalled()) && (numOfStalls<2) && (currentState == AutoStates.PICKUP_STACK_PIXELS || currentState == AutoStates.RETRY_STACK)) {
            saveTransition = subTransition;
            subTransition = 0;
            targetState = currentState;
            currentState = AutoStates.IS_STALLED;
        }

        telemetry .addData("Current state", currentState);
        telemetry.addData("Sub transition", subTransition);
        telemetry.update();
    }

    private void update() {
        driveTrain.update();

        delivery.update();
        slideLimit.update();
        slides.update();
        leftBeam.update();
        rightBeam.update();
        colorSensor.update();

        intakeProcessor.update();
        if (intakeProcessor.hasTwoPixel() || intakeProcessor.hasOnePixel()) {
            blinkin.setLEDColors(intakeProcessor.getLeftPixel(), intakeProcessor.getRightPixel());
        }
    }

    private boolean timerAt(double targetMS) {
        return timer.milliseconds() > targetMS;
    }

    private void buildPaths() {
        preloadDeliveryBackdropLeft = driveTrain.trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(48, 39, Math.toRadians(180)))
                .back(1)
                .build();

        preloadDeliveryBackdropCenter = driveTrain.trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(48, 32.5, Math.toRadians(180)))
                .back(1)
                .build();

        preloadDeliveryBackdropRight = driveTrain.trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(48, 27.5, Math.toRadians(180)))
                .back(1)
                .build();
        preloadDeliveryLeft = driveTrain.trajectorySequenceBuilder(preloadDeliveryBackdropLeft.end())
                .lineToConstantHeading(new Vector2d(29, 34))
                .build();

        preloadDeliveryCenter = driveTrain.trajectorySequenceBuilder(preloadDeliveryBackdropCenter.end())
                .lineToConstantHeading(new Vector2d(22,24))
                .build();

        preloadDeliveryRight = driveTrain.trajectorySequenceBuilder(preloadDeliveryBackdropRight.end())
                .lineToLinearHeading(new Pose2d(6, 33, Math.toRadians(180)))
                .build();

        toCommonPathCenter = driveTrain.trajectorySequenceBuilder(preloadDeliveryCenter.end())
                .lineToLinearHeading(new Pose2d(35, 13, Math.toRadians(180)))
                .build();

        toCommonPathLeft = driveTrain.trajectorySequenceBuilder(preloadDeliveryLeft.end())
                .back(3)
                .lineToLinearHeading(new Pose2d(42, 15, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(35, 13, Math.toRadians(180)))
                .build();
        toCommonPathRight = driveTrain.trajectorySequenceBuilder(preloadDeliveryRight.end())
                .back(6)
                .lineToLinearHeading(new Pose2d(22,24, Math.toRadians(180)))
                .build();

        toStackCenter = driveTrain.trajectorySequenceBuilder(toCommonPathCenter.end())
                .lineToLinearHeading(new Pose2d(0, 10, Math.toRadians(180)))
                .setReversed(false)
                .lineToLinearHeading(new Pose2d(-63, 9, Math.toRadians(180)))
                .build();
        toStackLeft = driveTrain.trajectorySequenceBuilder(toCommonPathCenter.end())
                .lineToLinearHeading(new Pose2d(0, 10, Math.toRadians(180)))
                .setReversed(false)
                .lineToLinearHeading(new Pose2d(-63, 9, Math.toRadians(180)))
                .build();
        toStackRight = driveTrain.trajectorySequenceBuilder(toCommonPathRight.end())
                .lineToLinearHeading(new Pose2d(0, 8, Math.toRadians(180)))
                .setReversed(false)
                .lineToLinearHeading(new Pose2d(-62.5, 6, Math.toRadians(180)))
                .build();

        backToKnownPosition = driveTrain.trajectorySequenceBuilder(new Pose2d(-65, 15, Math.toRadians(180)))
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(-35, 12, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(20, 8, Math.toRadians(180)))
                .build();

        backToKnownPositionRight = driveTrain.trajectorySequenceBuilder(new Pose2d(-65, 15, Math.toRadians(180)))
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(-35, 12, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(10, 8, Math.toRadians(180)))
                .build();


        stackDeliveryBackdrop = driveTrain.trajectorySequenceBuilder(backToKnownPosition.end())
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(42, 32, Math.toRadians(180)))
                .build();

        stackDeliveryBackdropRight = driveTrain.trajectorySequenceBuilder(backToKnownPositionRight.end())
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(42, 32, Math.toRadians(180)))
                .build();

        park = driveTrain.trajectorySequenceBuilder(backToKnownPosition.end())
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(53, 10, Math.toRadians(0)))
                .build();

        park2 = driveTrain.trajectorySequenceBuilder(stackDeliveryBackdrop.end())
                .forward(2)
                .strafeLeft(17)
                .back(5)
                .build();
    }
}
