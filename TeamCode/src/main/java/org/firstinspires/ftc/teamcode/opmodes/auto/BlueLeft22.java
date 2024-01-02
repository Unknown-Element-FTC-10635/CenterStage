package org.firstinspires.ftc.teamcode.opmodes.auto;

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

public class BlueLeft22 extends OpMode {
    public enum AutoStates {
        START,
        SCORE_PURPLE_PRELOAD,
        SCORE_YELLOW_PRELOAD,
        PICKUP_STACK_PIXELS,
        SCORE_STACK_PIXELS,
        PARK,
        WAIT_ARRIVAL,
        SLIDE_TIMEOUT,
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

    private TrajectorySequence preloadDelivery, preloadDeliveryBackdrop, toStack, stackDeliveryBackdrop, park;
    private PropProcessor.Spikes spikePosition;
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

        timer = new ElapsedTime();
        timer.startTime();
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

        spikePosition = webcam.propProcessor.getSpikePosition();
        webcam.stopWebcam();
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
                        slides.setHeight(Slides.SlidesHeights.SECOND_LEVEL);
                        delivery.setDeliveryState(Delivery.DeliveryState.SCORE_AUTO);

                        timer.reset();
                        subTransition++;
                        break;
                    case 1:
                    case 7:
                        if (slides.atTargetPosition()) {
                            timer.reset();
                            subTransition++;
                        } else if (timerAt(1000)) {
                            subTransition = 0;
                            targetState = AutoStates.SCORE_YELLOW_PRELOAD;
                            currentState = AutoStates.SLIDE_TIMEOUT;
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
                    case 5:
                        if (timerAt(500)) {
                            subTransition++;
                        }

                        break;
                    case 6:
                        delivery.setDeliveryState(Delivery.DeliveryState.TRANSITION_2);
                        slides.setHeight(Slides.SlidesHeights.BASE);

                        subTransition++;
                        break;
                    case 8:
                        intake.off();
                        intake.setServoPosition(Intake.IntakeState.STACK_HIGH);
                        driveTrain.followTrajectorySequenceAsync(toStack);

                        subTransition = 0;
                        targetState = AutoStates.PICKUP_STACK_PIXELS;
                        currentState = AutoStates.WAIT_ARRIVAL;
                        break;
                }

                break;
            case PICKUP_STACK_PIXELS:
                break;
            case SCORE_STACK_PIXELS:
                break;
            case PARK:
                break;
            case WAIT_ARRIVAL:
                if (!driveTrain.isBusy()) {
                    currentState = targetState;
                }

                break;
            case SLIDE_TIMEOUT:
                switch (subTransition) {
                    case 0:
                        slides.setPidEnabled(false);
                        slides.manual(-0.3);

                        subTransition++;
                        break;
                    case 1:
                        if (slideLimit.isPressed()) {
                            slides.manual(0);
                            slides.resetEncoders();

                            subTransition++;
                        }

                        break;
                    case 2:
                        slides.setHeight(Slides.SlidesHeights.BASE);
                        slides.setPidEnabled(true);

                        subTransition = 0;
                        currentState = targetState;
                        break;
                }

                break;
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
}
