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
import org.firstinspires.ftc.teamcode.utils.AutoStates;
import org.firstinspires.ftc.teamcode.utils.CurrentOpmode;
import org.firstinspires.ftc.teamcode.utils.PixelColors;
import org.firstinspires.ftc.teamcode.vision.IntakeProcessor;
import org.firstinspires.ftc.teamcode.vision.PropProcessor;

@Autonomous(name = "Rams - BLUE (Backboard) Truss - 2+2", group = "blue")
public class BlueLeft22TrussRams extends OpMode {
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
    private TrajectorySequence  preloadDelivery, preloadDeliveryBackdrop, toCommonPath, toStack, backToKnownPosition, stackDeliveryBackdrop, park, park2;
    private Pose2d startPose;
    private AutoStates currentState, targetState;
    private int subTransition;
    private int tries;
    private boolean tryToScore;

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
                        intake.reverse(0.6);
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
                        delivery.setDeliveryState(Delivery.DeliveryState.SCORE_PRELOAD);
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
                        timer.reset();
                        subTransition++;
                        break;
                    case 1:
                        claw.setClawState(Claw.ClawState.OPEN_AUTO);

                        subTransition++;
                        timer.reset();
                        break;
                    case 2:
                        if (timerAt(300)) {
                            timer.reset();
                            subTransition++;
                        }

                        break;
                    case 3:
                        delivery.setDeliveryState(Delivery.DeliveryState.INTAKE_HOLD);
                        intake.setServoPosition(Intake.IntakeState.GROUND);

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

                        if (tries < 1) {
                            currentState = AutoStates.RETRY_STACK;
                        } else {
                            tryToScore = intakeProcessor.hasTwoPixel();
                            targetState = AutoStates.DRIVE_THROUGH_BARRIER;
                        }
                        break;
                }
                break;
            case RETRY_STACK:
                switch (subTransition) {
                    case 0:
                        tries++;
                        intake.setServoPosition(Intake.IntakeState.GROUND);
                        intake.reverse(.35);
                        driveTrain.setMotorPowers(-0.75, 0.75, -0.75, 0.75);

                        timer.reset();
                        subTransition++;
                        break;
                    case 1:
                    case 3:
                        if (timerAt(450)) {
                            subTransition++;
                        }

                        break;
                    case 2:
                        driveTrain.setMotorPowers(0,0,0,0);
                        intake.on();
                        timer.reset();
                        subTransition++;
                        break;
                    case 4:
                        timer.reset();
                        subTransition = 0;
                        if (tries == 1) {
                            intake.setServoPosition(Intake.IntakeState.GROUND);
                        }
                        currentState = AutoStates.PICKUP_STACK_PIXELS;
                        break;
                }
                break;
            case DRIVE_THROUGH_BARRIER:
                switch (subTransition) {
                    case 0:
                        intake.setServoPosition(Intake.IntakeState.STACK_HIGH);
                        intake.on();

                        subTransition++;
                        break;
                    case 1:
                        driveTrain.followTrajectorySequenceAsync(backToKnownPosition);
                        intake.setServoPosition(Intake.IntakeState.STACK_MID);
                        delivery.setDeliveryState(Delivery.DeliveryState.INTAKE_PICKUP);
                        timer.reset();

                        subTransition++;
                        break;
                    case 2:
                        // Wait until we know the claw is parallel to the ground
                        if (timerAt(600)) {
                            subTransition++;
                        }

                        break;
                    case 3:
                        // Close claw onto the pixels
                        claw.setClawState(Claw.ClawState.CLOSED);
                        timer.reset();

                        subTransition++;
                        break;
                    case 4:
                        // Wait <milliseconds> so the physical servo has time to actually move
                        if (timerAt(725)) {
                            subTransition++;
                        }

                        break;
                    case 5:
                        // Move to safe transition point to avoid the cross-beam
                        delivery.setDeliveryState(Delivery.DeliveryState.INTAKE_HOLD);
                        intake.off();
                        timer.reset();

                        subTransition++;
                        break;
                    case 6:
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
                        slides.setHeight(Slides.SlidesHeights.SECOND_LEVEL);
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

        if ((currentState == AutoStates.PICKUP_STACK_PIXELS || currentState == AutoStates.RETRY_STACK) && intakeProcessor.hasTwoPixel()) {
            tryToScore = true;
            subTransition = 0;
            currentState = AutoStates.DRIVE_THROUGH_BARRIER;
        } else if (currentState == AutoStates.DRIVE_THROUGH_BARRIER && intakeProcessor.hasTwoPixel()){
            tryToScore = true;
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
            blinkin.setLEDColors(intakeProcessor.getLeftPixel(), intakeProcessor.getRightPixel());
        }
    }

    private boolean timerAt(double targetMS) {
        return timer.milliseconds() > targetMS;
    }

    private void buildPaths() {
        preloadDeliveryLeft = driveTrain.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(14, 53))
                .lineToLinearHeading(new Pose2d(22,36, Math.toRadians(260)))
                .back(5)
                .build();

        preloadDeliveryCenter = driveTrain.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(12, 33))
                .build();

        preloadDeliveryRight = driveTrain.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(15, 52))
                .lineToLinearHeading(new Pose2d(-1, 33, Math.toRadians(180)))
                .back(12)
                .build();

        preloadDeliveryBackdropLeft = driveTrain.trajectorySequenceBuilder(preloadDeliveryLeft.end())
                .back(10)
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(48, 36, Math.toRadians(180)))
                .back(5)
                .build();

        preloadDeliveryBackdropCenter = driveTrain.trajectorySequenceBuilder(preloadDeliveryCenter.end())
                .back(7)
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(49, 32, Math.toRadians(180)))
                .back(3)
                .build();

        preloadDeliveryBackdropRight = driveTrain.trajectorySequenceBuilder(preloadDeliveryRight.end())
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(48, 24, Math.toRadians(180)))
                .back(5)
                .build();

        toStack = driveTrain.trajectorySequenceBuilder(preloadDeliveryBackdropCenter.end())
                .splineTo(new Vector2d(13, 58), Math.toRadians(180))
                .lineToConstantHeading(new Vector2d(-38, 59))
                .splineTo(new Vector2d(-60, 40), Math.toRadians(180))
                .build();

        backToKnownPosition = driveTrain.trajectorySequenceBuilder(new Pose2d(-60, 37, Math.toRadians(180)))
                .setReversed(true)
                .lineToConstantHeading(new Vector2d(-47, 47))
                .lineToConstantHeading(new Vector2d(-40 , 59))
                .lineToConstantHeading(new Vector2d(23, 57))
                .build();

        stackDeliveryBackdrop = driveTrain.trajectorySequenceBuilder(backToKnownPosition.end())
                .setReversed(true)
                .splineTo(new Vector2d(55, 60), Math.toRadians(0))
                .turn(Math.toRadians(180))
                .build();

        park = driveTrain.trajectorySequenceBuilder(backToKnownPosition.end())
                .setReversed(true)
                .lineTo(new Vector2d(58, 60))
                .build();
    }
}
