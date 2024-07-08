package org.firstinspires.ftc.teamcode.opmodes.auto.CRI;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.fasterxml.jackson.databind.ser.std.ObjectArraySerializer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.apache.commons.math3.util.BigReal;
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

@Autonomous(name = "🟦 left 22", group = "blue")
public class CRI_BlueLeft22 extends OpMode {
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

    private TrajectorySequence path, park, leftPath, centerPath, rightPath, leftPark, centerPark, rightPark, toStack, backToBackboard;
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

        startPose = new Pose2d( 33, 61, Math.toRadians(275));
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
                        delivery.setDeliveryState(Delivery.DeliveryState.TRANSITION_1);
                        intake.setServoPosition(Intake.IntakeState.PRELOAD);
                        driveTrain.followTrajectorySequenceAsync(path);
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

                        subTransition = 0;
                        currentState = AutoStates.SCORE_PURPLE_PRELOAD;
                        break;

                }
                break;

            case SCORE_PURPLE_PRELOAD:
                switch (subTransition){
                    case 0:
                        if (timerAt(2500)) {
                            subTransition++;
                        }
                        break;
                    case 1:
                        intake.setServoPosition(Intake.IntakeState.STACK_HIGH);

                        timer.reset();
                        subTransition++;
                        break;
                    case 2:
                        if (timerAt(999)) {
                            subTransition++;
                        }
                        break;
                    case 3:
                        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                        intake.setServoPosition(Intake.IntakeState.STACK_HIGH);
                        slides.setHeight(Slides.SlidesHeights.PRELOAD);
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
                        if (timerAt(2)) {
                            subTransition++;
                        }
                        break;
                    case 2:
                        claw.setClawState(Claw.ClawState.OPEN_AUTO);

                        subTransition++;
                        timer.reset();
                        break;
                    case 3:
                        if (timerAt(600)) {
                            subTransition++;
                        }
                        break;
                    case 4:
                        delivery.setDeliveryState(Delivery.DeliveryState.INTAKE_HOLD);
                        claw.setClawState(Claw.ClawState.OPEN_INTAKE);
                        intake.setServoPosition(Intake.IntakeState.STACK_HIGH);
                        slides.setHeight(Slides.SlidesHeights.BASE);
                        intake.reverse(.4);
                        driveTrain.followTrajectorySequence(toStack);
                        subTransition = 0;
                        currentState = AutoStates.WAIT_ARRIVAL;
                        targetState = AutoStates.PICKUP_STACK_PIXELS;
                        timer.reset();
                        break;
                }
                break;

            case PICKUP_STACK_PIXELS:
                switch(subTransition){
                    case 0:
                        intake.setServoPosition(Intake.IntakeState.STACK_MID);
                        intake.on();
                        timer.reset();
                        subTransition++;
                        break;
                    case 1:
                        if(timerAt(2000) || intakeProcessor.hasOnePixel()){
                            intake.setServoPosition(Intake.IntakeState.RYANS_SMART);
                            timer.reset();
                            subTransition++;
                        }
                        break;
                    case 2:
                        if( timerAt(3000) ||intakeProcessor.hasTwoPixel()){
                            intake.setServoPosition(Intake.IntakeState.STACK_HIGH);
                            intake.on();
                            driveTrain.followTrajectorySequenceAsync(backToBackboard);
                            timer.reset();
                            subTransition++;
                        }
                        break;
                    case 3:
                        if(timerAt(700)){
                            delivery.setDeliveryState(Delivery.DeliveryState.INTAKE_PICKUP);
                            intake.reverse(.3);
                            timer.reset();
                            subTransition++;
                        }
                        break;
                    case 4:
                        if (timerAt(900)){
                            intake.reverse(.8);
                            claw.setClawState(Claw.ClawState.CLOSED);
                            timer.reset();
                            subTransition++;
                        }
                        break;
                    case 5:
                        if(timerAt(300)){
                            subTransition++;
                        }
                        break;

                    case 6:
                        delivery.setDeliveryState(Delivery.DeliveryState.INTAKE_HOLD);
                        timer.reset();
                        subTransition++;
                        break;
                    case 7:
                        if(timerAt(500)){
                            intake.off();
                            delivery.setDeliveryState(Delivery.DeliveryState.TRANSITION_1);
                            subTransition=0;
                            currentState = AutoStates.WAIT_ARRIVAL;
                            targetState = AutoStates.SCORE_STACK_PIXELS;
                        }
                        break;
                }
                break;
            case SCORE_STACK_PIXELS:
                switch (subTransition){
                    case 0:
                        slides.setHeight(Slides.SlidesHeights.HIGHEST_LEVEL);
                        subTransition++;
                        timer.reset();
                        break;
                    case 1:
                        if(timerAt(1000) || slides.atTargetPosition()){
                            delivery.setDeliveryState(Delivery.DeliveryState.SCORE_NO_OUT);
                            timer.reset();
                            subTransition++;

                        }
                        break;
                    case 2:
                        if(timerAt(500)){
                            claw.setClawState(Claw.ClawState.OPEN_SCORE);
                            timer.reset();
                            subTransition++;
                        }
                        break;
                    case 3:
                        if(timerAt(400)){
                            delivery.setDeliveryState(Delivery.DeliveryState.TRANSITION_1);
                            timer.reset();
                            subTransition++;
                        }
                        break;
                    case 4:
                        if(timerAt(400)){
                            slides.setHeight(Slides.SlidesHeights.BASE);
                            timer.reset();
                            subTransition++;
                        }
                        break;
                    case 5:
                        currentState = AutoStates.DONE;
                        break;



                }
                break;



            case DONE:
                intake.off();
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
        telemetry.addData("One pixel", intakeProcessor.hasOnePixel());
        telemetry.addData("two pixel", intakeProcessor.hasTwoPixel());
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
                .setReversed(true)
                .forward(5)
                .lineToLinearHeading(new Pose2d(49, 22,  Math.toRadians(180)))
                //drop yellow
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(76, 34, Math.toRadians(180)))
                .build();

        leftPath = driveTrain.trajectorySequenceBuilder(startPose)
                //drop Purple
                .setReversed(true)
                .forward(5)
                .lineToLinearHeading(new Pose2d(62, 30,  Math.toRadians(180)))
                //drop yellow
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(76, 42, Math.toRadians(180)))
                .build();
        rightPath = driveTrain.trajectorySequenceBuilder(startPose)
                //drop Purple
                .setReversed(true)
                .forward(5)
                .lineToLinearHeading(new Pose2d(32, 32,  Math.toRadians(180)))
                //drop yellow
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(76, 29, Math.toRadians(180)))
                .build();




        centerPark =  driveTrain.trajectorySequenceBuilder(centerPath.end())
                .setReversed(false)
                .forward(8)
                .strafeRight(25)
                .back(8)
                .build();

        leftPark =  driveTrain.trajectorySequenceBuilder(leftPath.end())
                .setReversed(false)
                .forward(8)
                .strafeRight(17)
                .back(8)
                .build();

        rightPark = driveTrain.trajectorySequenceBuilder(rightPath.end())
                .setReversed(false)
                .forward(8)
                .strafeRight(32)
                .back(8)
                .build();

        toStack = driveTrain.trajectorySequenceBuilder(centerPath.end())
                .lineToLinearHeading(new Pose2d(45, 11, Math.toRadians(175)))
                .lineToLinearHeading(new Pose2d(-10, 13, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(-21, 11, Math.toRadians(235)))
                .build();

        backToBackboard = driveTrain.trajectorySequenceBuilder(toStack.end())
                .lineToLinearHeading(new Pose2d(-6, 12, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(55, 11, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(75, 3, Math.toRadians(175)))

                .build();








    }
}
