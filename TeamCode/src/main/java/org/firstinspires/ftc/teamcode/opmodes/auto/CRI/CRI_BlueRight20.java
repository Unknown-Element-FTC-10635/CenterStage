package org.firstinspires.ftc.teamcode.opmodes.auto.CRI;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.apache.commons.math3.geometry.enclosing.SupportBallGenerator;
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

@Autonomous(name = "🟦 Stack 2+0", group = "blue")
public class CRI_BlueRight20 extends OpMode {
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

    private TrajectorySequence path, park, leftPath, centerPath, rightPath, leftPark, centerPark, rightPark;
    private Pose2d startPose;
    private AutoStates currentState, targetState;
    private int subTransition, saveTransition;
    private int tries;
    private boolean tryToScore;
    private int numOfStalls;
    private PropProcessor.Spikes spikeNum;
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

        startPose = new Pose2d(-12, 61, Math.toRadians(270));
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
        spikeNum = spikePosition;

        switch (spikePosition) {
            case LEFT:
                path = leftPath;
                park = leftPark;


                break;
            case CENTER:
                path = centerPath ;
                park = centerPark;
                break;
            case RIGHT:
                path = rightPath;
                park = rightPark;
                break;
        }


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
                        timer.reset();
                        subTransition++;
                        break;
                    case 1:
                        if(timerAt(4000)){
                            subTransition++;
                        }
                        break;
                    case 2:
                        delivery.setDeliveryState(Delivery.DeliveryState.INTAKE_HOLD);
                        intake.setServoPosition(Intake.IntakeState.PRELOAD);
                        driveTrain.followTrajectorySequenceAsync(path);
                        timer.reset();
                        subTransition++;
                        break;
                    case 3:
                        if (timerAt(1000)) {
                            timer.reset();
                            subTransition++;
                        }
                        break;
                    case 4:

                        subTransition = 0;
                        currentState = AutoStates.SCORE_PURPLE_PRELOAD;
                        break;

                }
                break;

            case SCORE_PURPLE_PRELOAD:
                switch (subTransition){
                    case 0:
                        switch (spikeNum){
                            case LEFT:
                            case CENTER:
                                if(timerAt(1200)){
                                    subTransition++;
                                }
                                break;
                            case RIGHT:
                                if(timerAt(2000)){
                                    subTransition++;
                                }
                                break;

                        }
                        break;
                    case 1:
                        intake.setServoPosition(Intake.IntakeState.STACK_HIGH);

                        timer.reset();
                        subTransition++;
                        break;
                    case 2:
                        switch (spikeNum){
                            case LEFT:
                                if(timerAt(11000)){
                                    subTransition++;
                                }
                                break;
                            case CENTER:
                                if(timerAt(7000)){
                                    subTransition++;
                                }
                                break;
                            case RIGHT:
                                if(timerAt(8000)){
                                    subTransition++;
                                }
                                break;

                        }
                        break;
                    case 3:
                        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                        intake.setServoPosition(Intake.IntakeState.STACK_HIGH);
                        slides.setHeight(Slides.SlidesHeights.FOURTH_LEVEL);
                        delivery.setDeliveryState(Delivery.DeliveryState.SCORE_NO_OUT);

                        subTransition = 0;
                        currentState = AutoStates.WAIT_ARRIVAL;
                        targetState = AutoStates.SCORE_YELLOW_PRELOAD;
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
                        if (timerAt(200)) {
                            subTransition++;
                        }
                        break;
                    case 2:
                        claw.setClawState(Claw.ClawState.OPEN_AUTO);

                        subTransition++;
                        timer.reset();
                        break;
                    case 3:
                        if (timerAt(1000)) {
                            subTransition++;
                        }
                        break;
                    case 4:
                        delivery.setDeliveryState(Delivery.DeliveryState.TRANSITION_2);
                        claw.setClawState(Claw.ClawState.OPEN_INTAKE);
                        intake.setServoPosition(Intake.IntakeState.STACK_HIGH);
                        subTransition++;

                        intake.off();
                        timer.reset();
                        break;
                    case 5:
                        if(timerAt(500)){
                            subTransition++;
                        }
                        break;
                    case 6:
                        slides.setHeight(Slides.SlidesHeights.BASE);
                        delivery.setDeliveryState(Delivery.DeliveryState.INTAKE_HOLD);
                        timer.reset();
                        subTransition++;
                        break;
                    case 7:
                        if(timerAt(400)){
                            subTransition++;
                        }
                        break;
                    case 8:
                        driveTrain.followTrajectorySequenceAsync(park);
                        subTransition = 0;
                        currentState = AutoStates.DONE;
                }
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
        centerPath = driveTrain.trajectorySequenceBuilder(startPose)
                //drop Purple
                .setReversed(false)
                .lineToLinearHeading(new Pose2d(-11, 33, Math.toRadians(270)))
                .setReversed(true)
                .splineTo(new Vector2d(-6, 58), Math.toRadians(0))
                .setReversed(false)
                .lineToLinearHeading(new Pose2d(80, 58, Math.toRadians(180)))
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(124, 33, Math.toRadians(180)))
                .build();

        leftPath = driveTrain.trajectorySequenceBuilder(startPose)
                .setReversed(false)
                .lineToLinearHeading(new Pose2d(-12, 35, Math.toRadians(0)))
                .forward(3)
                .back(6)
                .setReversed(true)
                .splineTo(new Vector2d(2, 59), Math.toRadians(0))
                .setReversed(false)
                .lineToLinearHeading(new Pose2d(80, 60, Math.toRadians(180)))
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(124, 50, Math.toRadians(180)))
                .build();

        rightPath = driveTrain.trajectorySequenceBuilder(startPose)
                .setReversed(false)
                .forward(4)
                .lineToLinearHeading(new Pose2d(-24, 45, Math.toRadians(270)))
                .lineToLinearHeading(new Pose2d(-5, 58, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(80, 58, Math.toRadians(180)))
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(124, 29, Math.toRadians(180)))
                .build();


        centerPark =  driveTrain.trajectorySequenceBuilder(centerPath.end())
                .setReversed(false)
                .forward(8)
                .strafeLeft(25)
                .build();

        leftPark =  driveTrain.trajectorySequenceBuilder(leftPath.end())
                .setReversed(false)
                .forward(8)
                .strafeLeft(25)
                .build();

        rightPark = driveTrain.trajectorySequenceBuilder(rightPath.end())
                .setReversed(false)
                .forward(8)
                .strafeLeft(15)
                .build();






    }
}
