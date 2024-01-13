package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Airplane;
import org.firstinspires.ftc.teamcode.hardware.BreakBeam;
import org.firstinspires.ftc.teamcode.hardware.Claw;
import org.firstinspires.ftc.teamcode.hardware.Delivery;
import org.firstinspires.ftc.teamcode.hardware.Hang;
import org.firstinspires.ftc.teamcode.hardware.Slides;
import org.firstinspires.ftc.teamcode.hardware.DriveTrain;
import org.firstinspires.ftc.teamcode.hardware.Intake;
import org.firstinspires.ftc.teamcode.hardware.LimitSwitch;
import org.firstinspires.ftc.teamcode.hardware.Webcam;
import org.firstinspires.ftc.teamcode.utils.CurrentOpmode;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.hardware.GamepadEx;
import org.firstinspires.ftc.teamcode.vision.IntakeProcessor;

@TeleOp
public class UKTeleOp extends OpMode {
    private GamepadEx controller1, controller2;
    private BreakBeam leftBeam, rightBeam;
    private LimitSwitch slideLimit;
    private DriveTrain driveTrain;
    private Airplane airplane;
    private Delivery delivery;
    private Webcam webcam;
    private Intake intake;
    private Slides slides;
    private Claw claw;
    private Hang hang;

    private IntakeProcessor processor;

    private RobotState robotState;
    private boolean toBackboard;
    private int driveDeliveryTransition;

    private ElapsedTime transitionTimer, matchTimer;
    private boolean leftRumble, rightRumble, fullRumble;
    private int targetBackboardLevel = 0;
    private boolean stackIntakeToggle;
    private IntakeProcessor.PixelColors leftIntake, rightIntake;

    @Override
    public void init() {
        CurrentOpmode.setCurrentOpmode(CurrentOpmode.OpMode.TELEOP);

        controller1 = new GamepadEx(gamepad1);
        controller2 = new GamepadEx(gamepad2);

        slideLimit = new LimitSwitch(hardwareMap, "slide limit");
        rightBeam = new BreakBeam(hardwareMap, "right break");
        leftBeam = new BreakBeam(hardwareMap, "left break");
        driveTrain = new DriveTrain(hardwareMap, controller1);
        airplane = new Airplane(hardwareMap);
        delivery = new Delivery(hardwareMap);
        processor = new IntakeProcessor();
        webcam = new Webcam(hardwareMap, processor, "intake webcam");
        intake = new Intake(hardwareMap);
        slides = new Slides(hardwareMap);
        claw = new Claw(hardwareMap);
        hang = new Hang(hardwareMap);
        transitionTimer = new ElapsedTime();
        transitionTimer.startTime();

        matchTimer = new ElapsedTime();
        matchTimer.startTime();

        robotState = RobotState.DRIVE;

        telemetry.addLine("Ready to start");
    }

    @Override
    public void start() {
        intake.setServoPosition(Intake.IntakeState.STACK_MID);
    }

    @Override
    public void loop() {
        update();

        if (controller1.leftTriggerPressedLiterallyAtAllRisingEdge()) {
            driveTrain.toggleSpeedMultiplier();
        }

        if (controller1.leftTriggerPressedLiterallyAtAllFallingEdge()) {
            driveTrain.toggleSpeedMultiplier();
        }

        switch (robotState) {
            // Drive -> Intake
            case DRIVE_INTAKE_TRANSITION:
                delivery.setDeliveryState(Delivery.DeliveryState.INTAKE_HOLD);
                slides.setHeight(Slides.SlidesHeights.BASE);
                claw.setClawState(Claw.ClawState.OPEN_INTAKE);
                intake.setServoPosition(Intake.IntakeState.GROUND);
                intake.on();

                fullRumble = false;
                toBackboard = true;
                robotState = RobotState.INTAKE;
                break;
            // For intake-ing new pixels
            case INTAKE:
                if (controller1.risingEdgeOf(GamepadEx.Buttons.CROSS)) {
                    driveDeliveryTransition = 0;
                    robotState = RobotState.INTAKE_DRIVE_TRANSITION;
                }

                if (controller1.risingEdgeOf(GamepadEx.Buttons.BUMPER_RIGHT)) {
                    intake.reverse();
                } else if (controller1.fallingEdgeOf(GamepadEx.Buttons.BUMPER_RIGHT)) {
                    intake.on();
                }

                if (controller1.risingEdgeOf(GamepadEx.Buttons.SQUARE)) {
                    stackIntakeToggle = !stackIntakeToggle;
                    if (stackIntakeToggle) {
                        intake.setServoPosition(Intake.IntakeState.STACK_HIGH);
                    } else {
                        intake.setServoPosition(Intake.IntakeState.GROUND);
                    }
                }

                if (leftIntake != IntakeProcessor.PixelColors.NONE && rightIntake != IntakeProcessor.PixelColors.NONE) {
                    gamepad1.rumble(250);
                    driveDeliveryTransition = 0;
                    robotState = RobotState.INTAKE_DRIVE_TRANSITION;
                }

                break;
            //  Intake -> Drive
            case INTAKE_DRIVE_TRANSITION:
                // Pick up pixels in the intake and move to a safe transition point
                switch (driveDeliveryTransition) {
                    case 0:
                        // Move into the holes in the pixel
                        delivery.setDeliveryState(Delivery.DeliveryState.INTAKE_PICKUP);
                        intake.setServoPosition(Intake.IntakeState.STACK_MID);
                        transitionTimer.reset();

                        driveDeliveryTransition++;
                        break;
                    case 1:
                        // Wait until we know the claw is parallel to the ground
                        if (transitionTimer.milliseconds() > 325) {
                            driveDeliveryTransition++;
                        }

                        break;
                    case 2:
                        // Close claw onto the pixels
                        claw.setClawState(Claw.ClawState.CLOSED);
                        transitionTimer.reset();

                        driveDeliveryTransition++;
                        break;
                    case 3:
                        // Wait <milliseconds> so the physical servo has time to actually move
                        if (transitionTimer.milliseconds() > 725) {
                            driveDeliveryTransition++;
                        }

                        break;
                    case 4:
                        // Move to safe transition point to avoid the cross-beam
                        delivery.setDeliveryState(Delivery.DeliveryState.INTAKE_HOLD);
                        intake.off();
                        transitionTimer.reset();

                        driveDeliveryTransition++;
                        break;
                    case 5:
                        if (transitionTimer.milliseconds() > 1000) {
                            driveDeliveryTransition++;
                        }

                        break;
                    case 6:
                        // Move to safe transition point that works for both intake and score
                        delivery.setDeliveryState(Delivery.DeliveryState.TRANSITION_2);
                        driveDeliveryTransition++;
                        transitionTimer.reset();
                        break;
                        // A pause to prevent spamming immediately from intake -> score, which
                        // smashes the pixels against the edge of the intake
                    case 7:
                        // Wait <milliseconds> so the physical servo has time to actually move
                        if (transitionTimer.milliseconds() > 500) {
                            driveDeliveryTransition++;
                        }

                        break;
                    case 8:
                        // Advance
                        robotState = RobotState.DRIVE;
                        break;
                }

                if (controller2.risingEdgeOf(GamepadEx.Buttons.TRIANGLE)) {
                    robotState = RobotState.DRIVE_INTAKE_TRANSITION;
                }

                break;
            // Mostly just a transition state, don't think anything special needs to be here
            case DRIVE:
                if (controller1.risingEdgeOf(GamepadEx.Buttons.CROSS)) {
                    // The cross over
                    if (toBackboard) {
                        driveDeliveryTransition = 0;
                        robotState = RobotState.DRIVE_SCORE_TRANSITION;
                    } else {
                        robotState = RobotState.DRIVE_INTAKE_TRANSITION;
                    }
                }

                if (controller1.risingEdgeOf(GamepadEx.Buttons.TRIANGLE)) {
                    // The cross over
                    if (!toBackboard) {
                        driveDeliveryTransition = 0;
                        robotState = RobotState.DRIVE_SCORE_TRANSITION;
                    } else {
                        robotState = RobotState.DRIVE_INTAKE_TRANSITION;
                    }
                }

                break;
            // Drive -> Score
            case DRIVE_SCORE_TRANSITION:
                switch (driveDeliveryTransition) {
                    case 0:
                        // Move to transition point between intake and score
                        slides.setHeight(Slides.SlidesHeights.levelFromInt(targetBackboardLevel));
                        transitionTimer.reset();

                        driveDeliveryTransition++;
                        delivery.setDeliveryState(Delivery.DeliveryState.SCORE);
                        break;
                    case 1:
                        // Wait until the slides are at that position
                        if (slides.atTargetPosition() || transitionTimer.milliseconds() > 750) {
                            driveDeliveryTransition++;
                        }

                        break;
                    case 2:
                        // Advance
                        robotState = RobotState.SCORE;
                        break;
                }

                if (controller1.risingEdgeOf(GamepadEx.Buttons.CROSS)) {
                    driveDeliveryTransition = 0;
                    robotState = RobotState.SCORE;
                }

                toBackboard = false;
                break;
            // For scoring on the backboard
            case SCORE:
                if (controller1.risingEdgeOf(GamepadEx.Buttons.BUMPER_RIGHT)) {
                    if (targetBackboardLevel < 3) {
                        targetBackboardLevel++;
                        slides.setHeight(Slides.SlidesHeights.levelFromInt(targetBackboardLevel));
                    }
                }

                if (controller1.risingEdgeOf(GamepadEx.Buttons.BUMPER_LEFT)) {
                    if (targetBackboardLevel > 0) {
                        targetBackboardLevel--;
                        slides.setHeight(Slides.SlidesHeights.levelFromInt(targetBackboardLevel));
                    }
                }

                if (controller1.risingEdgeOf(GamepadEx.Buttons.CROSS)) {
                    driveDeliveryTransition = 0;
                    robotState = RobotState.SCORE_DRIVE_TRANSITION;
                }

                break;
            // Score -> Drive
            case SCORE_DRIVE_TRANSITION:
                switch (driveDeliveryTransition) {
                    case 0:
                        claw.setClawState(Claw.ClawState.OPEN_SCORE);
                        transitionTimer.reset();

                        driveDeliveryTransition++;
                        break;
                    case 1:
                        if (transitionTimer.milliseconds() > 250) {
                            driveDeliveryTransition++;
                        }

                        break;
                    case 2:
                        delivery.setDeliveryState(Delivery.DeliveryState.TRANSITION_2);
                        transitionTimer.reset();

                        driveDeliveryTransition++;
                        break;
                    case 3:
                        if (transitionTimer.milliseconds() > 500) {
                            driveDeliveryTransition++;
                        }

                        break;
                    case 4:
                        // Move to transition point between intake and score
                        slides.setHeight(Slides.SlidesHeights.BASE);
                        delivery.setDeliveryState(Delivery.DeliveryState.TRANSITION_2);
                        transitionTimer.reset();

                        driveDeliveryTransition++;
                        break;
                    case 5:
                        // Wait until the slides are at that position
                        if (slides.atTargetPosition() || slideLimit.isPressed() || transitionTimer.milliseconds() > 2000) {
                            driveDeliveryTransition++;
                        }

                        break;
                    case 6:
                        // Advance
                        robotState = RobotState.DRIVE;
                        break;
                }


                break;
            case TRANSITION_ENDGAME:
                slides.setHeight(Slides.SlidesHeights.BASE);
                intake.off();
                hang.setHangState(Hang.HangState.UP);

                if (transitionTimer.milliseconds() > 250) {
                    robotState = RobotState.ENDGAME;
                }

                break;
            case ENDGAME:
                hang.motor(gamepad1.left_trigger - gamepad1.right_trigger);

                if(controller1.risingEdgeOf(GamepadEx.Buttons.DPAD_DOWN)){
                    hang.setHangState(Hang.HangState.DOWN);
                }

                if (controller1.risingEdgeOf(GamepadEx.Buttons.SQUARE)) {
                    airplane.launch();
                }

                break;
        }

        if (controller1.risingEdgeOf(GamepadEx.Buttons.DPAD_UP)) {
            robotState = RobotState.TRANSITION_ENDGAME;
            transitionTimer.reset();
        }

        if (slideLimit.isRisingEdge()) {
            slides.resetEncoders();
        }

        write();
    }

    private void update() {
        // Need to be the first things
        controller1.update();
        controller2.update();

        delivery.update();
        slideLimit.update();
        slides.update();
        //leftBeam.update();
        //rightBeam.update();

        leftIntake = processor.getLeftPixel();
        rightIntake = processor.getRightColor();
    }

    private void write() {
        driveTrain.writeTeleOp();

        telemetry.addData("Speed", driveTrain.getSpeedMultiplier());
        telemetry.addData("Delivery", delivery.getDeliveryState());
        telemetry.addData("Delivery Right Rotation", delivery.getRightRotationPosition());
        telemetry.addData("Slide Switch", slideLimit.isPressed());
        telemetry.addData("Transition Timer", transitionTimer.milliseconds());
        telemetry.addData("Slides Left Position", slides.getCurrentLeftPosition());
        telemetry.addData("Slides Right Position", slides.getCurrentRightPosition());
        telemetry.addData("Slide Left Power", slides.getCurrentLeftPower());
        telemetry.addData("Slide Right Power", slides.getCurrentRightPower());
        telemetry.addData("Slide Left Error", slides.getLeftError());
        telemetry.addData("Slide Right Error", slides.getRightError());
        telemetry.addData("Slide at Target Position", slides.atTargetPosition());
        telemetry.addData("Backboard Level", targetBackboardLevel);
        telemetry.addData("Left Intake Pixel", leftIntake);
        telemetry.addData("Right Intake Pixel", rightIntake);
        telemetry.addData("Robot", robotState);

        telemetry.addData("Loop time", matchTimer.milliseconds());
        matchTimer.reset();
    }
}
