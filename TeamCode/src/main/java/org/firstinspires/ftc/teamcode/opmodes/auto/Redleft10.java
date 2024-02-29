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

@Autonomous(name = "RED (Stack Side) - 1+0", group = "red")
public class Redleft10 extends OpMode {
    public enum AutoStates {
        START,
        SCORE_PURPLE_PRELOAD,
        WAIT_ARRIVAL,
        DONE,
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

    private TrajectorySequence  preloadDeliveryLeft, preloadDeliveryRight, preloadDeliveryCenter, backLeft, backRight, backCenter;
    private TrajectorySequence  preloadDelivery, back;
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

        startPose =  new Pose2d(-36, -62, Math.toRadians(90));
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
                back = backLeft;
                break;
            case CENTER:
                preloadDelivery = preloadDeliveryCenter;
                back = backCenter;
                break;
            case RIGHT:
                preloadDelivery = preloadDeliveryRight;
                back = backRight;
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
                        intake.reverse(.35);
                        timer.reset();

                        subTransition++;
                        break;
                    case 1:
                        if (timerAt(600)) {
                            subTransition++;
                        }

                        break;
                    case 2:
                        intake.off();
                        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                        delivery.setDeliveryState(Delivery.DeliveryState.INTAKE_HOLD);
                        driveTrain.followTrajectorySequenceAsync(back);

                        subTransition = 0;
                        targetState = AutoStates.DONE;
                        currentState = AutoStates.WAIT_ARRIVAL;
                        break;
                }
                break;
            case DONE:

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

        intakeProcessor.update();
        if (intakeProcessor.hasTwoPixel() || intakeProcessor.hasOnePixel()) {
            blinkin.setLEDColors(intakeProcessor.getLeftPixel(), intakeProcessor.getRightColor());
        }
    }

    private boolean timerAt(double targetMS) {
        return timer.milliseconds() > targetMS;
    }

    private void buildPaths() {
        preloadDeliveryCenter = driveTrain.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(-34, -25), Math.toRadians(80))
                .back(15)
                .build();


        preloadDeliveryRight = driveTrain.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(-45, -30), Math.toRadians(80))
                .back(8)
                .build();

        preloadDeliveryLeft = driveTrain.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(-30, -38), Math.toRadians(340))
                .build();

        backLeft = driveTrain.trajectorySequenceBuilder(preloadDeliveryLeft.end())
                .back(10)
                .build();

        backRight = driveTrain.trajectorySequenceBuilder(preloadDeliveryRight.end())
                .back(10)
                .build();

        backCenter = driveTrain.trajectorySequenceBuilder(preloadDeliveryCenter.end())
                .back(10)
                .build();



    }
}
