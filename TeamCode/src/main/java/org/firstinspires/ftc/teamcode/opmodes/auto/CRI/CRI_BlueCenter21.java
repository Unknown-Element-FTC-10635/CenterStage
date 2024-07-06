package org.firstinspires.ftc.teamcode.opmodes.auto.CRI;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Blinkin;
import org.firstinspires.ftc.teamcode.hardware.BreakBeam;
import org.firstinspires.ftc.teamcode.hardware.Claw;
import org.firstinspires.ftc.teamcode.hardware.Hang;
import org.firstinspires.ftc.teamcode.hardware.StackColorSensor;
import org.firstinspires.ftc.teamcode.hardware.Delivery;
import org.firstinspires.ftc.teamcode.hardware.Intake;
import org.firstinspires.ftc.teamcode.hardware.LimitSwitch;
import org.firstinspires.ftc.teamcode.hardware.Slides;
import org.firstinspires.ftc.teamcode.hardware.Webcam;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.utils.AutoStates;
import org.firstinspires.ftc.teamcode.utils.CurrentOpmode;
import org.firstinspires.ftc.teamcode.vision.IntakeProcessor;
import org.firstinspires.ftc.teamcode.vision.PropProcessor;
import org.opencv.core.Mat;

@Autonomous(name = "🟦 Center 21", group = "blue")
public class CRI_BlueCenter21 extends OpMode {
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

    private TrajectorySequence purple, pickupStack, throughBarrier, toBackboard;
    private Pose2d startPose;
    private AutoStates currentState, targetState;
    private int subTransition, saveTransition;
    private int tries;
    private boolean tryToScore;
    private int numOfStalls;
    @Override
    public void init() {
        CurrentOpmode.setCurrentOpmode(CurrentOpmode.OpMode.AUTO);

        slideLimit = new LimitSwitch(hardwareMap, "slide limit");
        colorSensor = new StackColorSensor(hardwareMap, true);
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

        startPose = new Pose2d( -34, 61, Math.toRadians(275));
        driveTrain.setPoseEstimate(startPose);
        buildPaths();

        timer = new ElapsedTime();
        timer.startTime();

        parkTimer = new ElapsedTime();

        currentState = AutoStates.START;
        numOfStalls = 0;
        subTransition = 0;
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
    }

    @Override
    public void loop() {
        update();

        switch (currentState) {
            case START:
                switch (subTransition){
                    case 0:
                        delivery.setDeliveryState(Delivery.DeliveryState.TRANSITION_1);
                        intake.setServoPosition(Intake.IntakeState.GROUND);
                        subTransition++;
                        break;
                    case 1:
                        subTransition = 0;
                        driveTrain.followTrajectorySequenceAsync(purple);
                        currentState = AutoStates.WAIT_ARRIVAL;
                        targetState = AutoStates.SCORE_PURPLE_PRELOAD;
                        break;
                }
                break;
            case SCORE_PURPLE_PRELOAD:
                switch (subTransition){
                    case 0:
                        delivery.setDeliveryState(Delivery.DeliveryState.SCORE_PURPLE);
                        timer.reset();
                        subTransition++;
                        break;
                    case 1:
                        if (timerAt(300)){
                            subTransition++;
                        }
                        break;
                    case 2:
                        claw.setClawState(Claw.ClawState.OPEN_SCORE);
                        driveTrain.followTrajectorySequenceAsync(pickupStack);
                        targetState = AutoStates.DRIVE_TO_STACK;
                        currentState = AutoStates.WAIT_ARRIVAL;
                        break;
                }
            case DRIVE_TO_STACK:
                currentState = AutoStates.DONE;
                break;
            case DONE:
                break;
            case WAIT_ARRIVAL:
                if(!driveTrain.isBusy()){
                    currentState = targetState;
                }
                break;
        }

        if (slideLimit.isRisingEdge()) {
            slides.resetEncoders();
        }

        if (intakeProcessor.hasTwoPixel()) {
            intake.off();
            intake.setServoPosition(Intake.IntakeState.STACK_HIGH);
            delivery.setDeliveryState(Delivery.DeliveryState.INTAKE_PICKUP);
            intake.reverse();
        }

//        if ((intake.isStalled()) && (numOfStalls<2) && (currentState == AutoStates.PICKUP_STACK_PIXELS || currentState == AutoStates.RETRY_STACK)) {
//            saveTransition = subTransition;
//            subTransition = 0;
//            targetState = currentState;
//            currentState = AutoStates.IS_STALLED;
//        }

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
        purple = driveTrain.trajectorySequenceBuilder(startPose)
                .setReversed(false)
                .lineToLinearHeading(new Pose2d(-32, 11, Math.toRadians(275)))
                .build();

        pickupStack = driveTrain.trajectorySequenceBuilder(purple.end())
                .setReversed(false)
                .lineToLinearHeading(new Pose2d(-37, 10, Math.toRadians(240)))
                .build();
        throughBarrier = driveTrain.trajectorySequenceBuilder(pickupStack.end())
                .setReversed(false)
                .lineToLinearHeading(new Pose2d(-26, 13, Math.toRadians(185)))
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(25, 13, Math.toRadians(185)))
                .build();
        toBackboard = driveTrain.trajectorySequenceBuilder(throughBarrier.end())
                .setReversed(false)
                .lineToLinearHeading(new Pose2d(54, 42, Math.toRadians(185)))
                .build();
    }
}
