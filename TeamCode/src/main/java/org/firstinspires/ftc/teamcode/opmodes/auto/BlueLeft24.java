package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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

@Disabled
@Autonomous(name = "BLUE (Backboard) - 2+4", group = "blue")
public class BlueLeft24 extends OpMode {
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

    private TrajectorySequence path;
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

        startPose = new Pose2d( 9, 61, Math.toRadians(275));
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
                        if (timerAt(2000)) {
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
                        intake.on(0.8);

                        subTransition = 0;
                        currentState = AutoStates.SCORE_YELLOW_PRELOAD;
                        break;
                }
                break;
            case SCORE_YELLOW_PRELOAD:
                switch (subTransition) {
                    case 0:
                        if (timerAt(2500)) {
                            subTransition++;
                        }
                        break;
                    case 1:
                        claw.setClawState(Claw.ClawState.OPEN_AUTO);

                        subTransition++;
                        timer.reset();
                        break;
                    case 2:
                        if (timerAt(600)) {
                            subTransition++;
                        }
                        break;
                    case 3:
                        delivery.setDeliveryState(Delivery.DeliveryState.INTAKE_HOLD);
                        claw.setClawState(Claw.ClawState.OPEN_INTAKE);
                        intake.setServoPosition(Intake.IntakeState.STACK_HIGH);
                        slides.setHeight(Slides.SlidesHeights.BASE);
                        subTransition = 0;
                        currentState = AutoStates.PICKUP_STACK_PIXELS;
                        timer.reset();
                        break;
                }
                break;

            case PICKUP_STACK_PIXELS:
                switch (subTransition){
                    case 0:
                        timer.reset();
                        subTransition++;
                        break;
                    case 1:
                        if (timerAt(4000)) {
                            subTransition++;
                        }
                        break;
                    case 2:
                        intake.setServoPosition(Intake.IntakeState.STACK_HIGH);
                        intake.reverse();
                        timer.reset();

                        subTransition++;
                        break;
                    case 3:
                        if (timerAt(500)){
                            subTransition++;
                        }
                        break;
                    case 4:
                        intake.reverse();
                        timer.reset();

                        subTransition++;
                        break;
                    case 5:
                        if (timerAt(500)){
                            subTransition++;
                        }
                        break;
                    case 6:
                        intake.setServoPosition(Intake.IntakeState.STACK_MID);
                        timer.reset();
                        intake.on(0.8);

                        subTransition++;
                        break;
                    case 7:
                        if (timerAt(199)){
                            subTransition++;
                            intake.on(0.8);
                            timer.reset();
                        }
                        break;
                    case 8:
                        if (timerAt(2500)){
                            subTransition++;
                        }
                        break;
                    case 9:
                        delivery.setDeliveryState(Delivery.DeliveryState.INTAKE_PICKUP);
                        timer.reset();
                        subTransition++;
                        break;
                    case 10:
                        // Wait until we know the claw is parallel to the ground
                        if (timerAt(900)) {
                            subTransition++;
                        }

                        break;
                    case 11:
                        // Close claw onto the pixels
                        claw.setClawState(Claw.ClawState.CLOSED);
                        timer.reset();


                        subTransition++;
                        break;
                    case 12:
                        if (timerAt(725)) {
                            subTransition++;
                        }

                        break;
                    case 13:
                        delivery.setDeliveryState(Delivery.DeliveryState.INTAKE_HOLD);
                        intake.off();
                        timer.reset();

                        subTransition++;
                        break;
                    case 14:
                        timer.reset();
                        subTransition =0;
                        currentState = AutoStates.SCORE_STACK_PIXELS;
                        break;
                }
                break;
            case SCORE_STACK_PIXELS:
                switch (subTransition) {
                    case 0:
                        if (timerAt(450)){
                            subTransition++;
                        }
                        break;
                    case 1:
                        subTransition++;
                        break;
                    case 2:
                        // Move to safe transition point that works for both intake and score
                        delivery.setDeliveryState(Delivery.DeliveryState.TRANSITION_1);
                        subTransition++;
                        timer.reset();
                        break;
                    case 3:
                        // Wait <milliseconds> so the physical servo has time to actually move
                        if (timerAt(1000)) {
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
                        if (timerAt(500)) {
                            timer.reset();
                            subTransition++;
                        }

                        break;
                    case 7:
                        claw.setClawState(Claw.ClawState.OPEN_SCORE);
                        timer.reset();

                        subTransition++;
                        break;
                    case 8:
                        if (timerAt(250)) {
                            subTransition++;
                        }

                        break;
                    case 9:
                        delivery.setDeliveryState(Delivery.DeliveryState.TRANSITION_2);
                        blinkin.clear();

                        timer.reset();
                        subTransition++;
                        break;
                    case 10:
                        if (timerAt(400)) {
                            subTransition++;
                        }

                        break;
                    case 11:
                        slides.setHeight(Slides.SlidesHeights.BASE);
                        intake.on(0.75);
                        timer.reset();
                        subTransition++;
                        break;
                    case 12:
                        if (timerAt(500)) {
                            subTransition++;
                        }
                        break;
                    case 13:
                        delivery.setDeliveryState(Delivery.DeliveryState.TRANSITION_1);
                        if (timerAt(300)) {
                            delivery.setDeliveryState(Delivery.DeliveryState.INTAKE_HOLD);
                            subTransition++;
                        }
                        break;
                    case 14:
                        intake.on(0.75);
                        timer.reset();
                        subTransition =0;
                        intake.setServoPosition(Intake.IntakeState.GROUND);
                        currentState = AutoStates.RETRY_STACK;
                        break;
                }
                break;
            case RETRY_STACK:
                switch (subTransition){
                    case 0:
                        if (timerAt(3000)) {
                            subTransition++;

                        }
                        break;
                    case 1:
                        intake.setServoPosition(Intake.IntakeState.GROUND);
                        timer.reset();

                        subTransition++;
                        break;
                    case 2:
                        if (timerAt(200)){
                            subTransition++;
                        }
                        break;
                    case 3:
                        intake.setServoPosition(Intake.IntakeState.GROUND);
                        timer.reset();
                        intake.on(0.8);

                        subTransition++;
                        break;
                    case 4:
                        if (timerAt(199)){
                            subTransition++;
                            intake.on(0.8);
                            timer.reset();
                        }
                        break;
                    case 5:
                        if (timerAt(600)){
                            subTransition++;
                        }
                        break;
                    case 6:
                        delivery.setDeliveryState(Delivery.DeliveryState.INTAKE_PICKUP);
                        timer.reset();
                        subTransition++;
                        break;
                    case 7:
                        // Wait until we know the claw is parallel to the ground
                        if (timerAt(600)) {
                            subTransition++;
                        }

                        break;
                    case 8:
                        // Close claw onto the pixels
                        claw.setClawState(Claw.ClawState.CLOSED);
                        timer.reset();


                        subTransition++;
                        break;
                    case 9:
                        if (timerAt(725)) {
                            subTransition++;
                        }

                        break;
                    case 10:
                        delivery.setDeliveryState(Delivery.DeliveryState.INTAKE_HOLD);
                        intake.off();
                        timer.reset();

                        subTransition++;
                        break;
                    case 11:
                        timer.reset();
                        subTransition =0;
                        currentState = AutoStates.SCORE_STACK_PIXELS_TWO;
                        break;
                }
                break;
            case SCORE_STACK_PIXELS_TWO:
                switch (subTransition) {
                    case 0:
                        if (timerAt(1000)){
                            subTransition++;
                        }
                        break;
                    case 1:
                        subTransition++;
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
                        if (timerAt(1400)) {
                            timer.reset();
                            subTransition++;
                        }

                        break;
                    case 7:
                        claw.setClawState(Claw.ClawState.OPEN_SCORE);
                        timer.reset();

                        subTransition++;
                        break;
                    case 8:
                        if (timerAt(250)) {
                            subTransition++;
                        }

                        break;
                    case 9:
                        delivery.setDeliveryState(Delivery.DeliveryState.TRANSITION_2);
                        blinkin.clear();

                        timer.reset();
                        subTransition++;
                        break;
                    case 10:
                        if (timerAt(400)) {
                            subTransition++;
                        }

                        break;
                    case 11:
                        slides.setHeight(Slides.SlidesHeights.BASE);
                        delivery.setDeliveryState(Delivery.DeliveryState.TRANSITION_1);

                        timer.reset();
                        subTransition =0;
                        currentState = AutoStates.DONE;
                        break;
                }
                break;

            case DONE:
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
        path = driveTrain.trajectorySequenceBuilder(startPose)
            //drop Purple
            .setReversed(true)
            .forward(5)
            .lineToLinearHeading(new Pose2d(23, 24,  Math.toRadians(180)))
            //drop yellow
            .setReversed(true)
            .lineToLinearHeading(new Pose2d(51, 34, Math.toRadians(180)))
            //drive to stack
            .lineToLinearHeading(new Pose2d(-5,35, Math.toRadians(180)))
            .lineToLinearHeading(new Pose2d(-63,35,Math.toRadians(180)))
            .back(7)
            .forward(7)
            .lineToLinearHeading(new Pose2d(0,35, Math.toRadians(180)))
            //drive back
            .lineToLinearHeading(new Pose2d(49, 35, Math.toRadians(180)))
            .waitSeconds(0.4)
            //drive back to stack
            .lineToLinearHeading(new Pose2d(-61,34, Math.toRadians(180)))
            .lineToLinearHeading(new Pose2d(0,35, Math.toRadians(180)))
            //drive back
            .lineToLinearHeading(new Pose2d(50, 37, Math.toRadians(180)))
            .build();

    }
}
