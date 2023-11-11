package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Airplane;
import org.firstinspires.ftc.teamcode.hardware.Claw;
import org.firstinspires.ftc.teamcode.hardware.Delivery;
import org.firstinspires.ftc.teamcode.hardware.Slides;
import org.firstinspires.ftc.teamcode.hardware.DriveTrain;
import org.firstinspires.ftc.teamcode.hardware.Intake;
import org.firstinspires.ftc.teamcode.hardware.LimitSwitch;
import org.firstinspires.ftc.teamcode.utils.CurrentOpmode;
import org.firstinspires.ftc.teamcode.utils.hardware.GamepadEx;

@TeleOp
public class UKTeleOp extends OpMode {
    public enum RobotState {
        DRIVE_INTAKE_TRANSITION,
        INTAKE,
        INTAKE_DRIVE_TRANSITION,
        DRIVE,
        DRIVE_SCORE_TRANSITION,
        SCORE,
        SCORE_DRIVE_TRANSITION
    }

    private GamepadEx controller1, controller2;
    private LimitSwitch intakeLimit, slideLimit;
    private DriveTrain driveTrain;
    private Airplane airplane;
    private Delivery delivery;
    private Intake intake;
    private Slides slides;
    private Claw claw;

    private RobotState robotState;
    private boolean toBackboard;
    private int driveDeliveryTransition;

    private ElapsedTime transitionTimer;
    private int targetBackboardLevel = 0;

    @Override
    public void init() {
        CurrentOpmode.setCurrentOpmode(CurrentOpmode.OpMode.TELEOP);

        controller1 = new GamepadEx(gamepad1);
        controller2 = new GamepadEx(gamepad2);

        intakeLimit = new LimitSwitch(hardwareMap, "intake limit");
        slideLimit = new LimitSwitch(hardwareMap, "slide limit");
        driveTrain = new DriveTrain(hardwareMap, controller1);
        airplane = new Airplane(hardwareMap);
        delivery = new Delivery(hardwareMap);
        intake = new Intake(hardwareMap);
        slides = new Slides(hardwareMap);
        claw = new Claw(hardwareMap);

        transitionTimer = new ElapsedTime();
        transitionTimer.startTime();

        robotState = RobotState.DRIVE;

        telemetry.addLine("Ready to start");
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
                intake.on();

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
                }

                if (controller1.fallingEdgeOf(GamepadEx.Buttons.BUMPER_RIGHT)) {
                    intake.on();
                }

                break;
            //  Intake -> Drive
            case INTAKE_DRIVE_TRANSITION:
                // Pick up pixels in the intake and move to a safe transition point
                switch (driveDeliveryTransition) {
                    case 0:
                        // Move into the holes in the pixel
                        delivery.setDeliveryState(Delivery.DeliveryState.INTAKE_PICKUP);
                        transitionTimer.reset();

                        driveDeliveryTransition++;
                        break;
                    case 1:
                        // Wait until we know the claw is parallel to the ground
                        if (intakeLimit.isPressed() || transitionTimer.milliseconds() > 750) {
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
                        // Wait <milliseconds> so the physical robot has time to actually close
                        if (transitionTimer.milliseconds() > 500) {
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
                        break;
                    case 7:
                        // Advance
                        robotState = RobotState.DRIVE;
                        break;
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
                        break;
                    case 1:
                        // Wait until the slides are at that position
                        if (slides.atTargetPosition() || transitionTimer.milliseconds() > 2000) {
                            driveDeliveryTransition++;
                        }

                        break;
                    case 2:
                        // Advance
                        delivery.setDeliveryState(Delivery.DeliveryState.SCORE);
                        robotState = RobotState.SCORE;
                        break;
                }

                toBackboard = false;
                break;
            // For scoring on the backboard
            case SCORE:
                if (controller1.risingEdgeOf(GamepadEx.Buttons.BUMPER_RIGHT)) {
                    if (targetBackboardLevel < 2) {
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
        }

        if (controller1.risingEdgeOf(GamepadEx.Buttons.DPAD_UP)) {
            airplane.launch();
        }

        write();
    }

    private void update() {
        // Need to be the first things
        controller1.update();
        controller2.update();

        intakeLimit.update();
        slideLimit.update();
        delivery.update();
        slides.update();
    }

    private void write() {
        driveTrain.writeTeleOp();

        telemetry.addData("Speed", driveTrain.getSpeedMultiplier());
        telemetry.addData("Delivery", delivery.getDeliveryState());
        telemetry.addData("Delivery Left Rotation", delivery.getLeftRotationPosition());
        telemetry.addData("Delivery Right Rotation", delivery.getRightRotationPosition());
        telemetry.addData("Intake Switch", intakeLimit.isPressed());
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
        telemetry.addData("Robot", robotState);
    }
}
