package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Airplane;
import org.firstinspires.ftc.teamcode.hardware.BackboardDetector;
import org.firstinspires.ftc.teamcode.hardware.Blinkin;
import org.firstinspires.ftc.teamcode.hardware.BreakBeam;
import org.firstinspires.ftc.teamcode.hardware.Claw;
import org.firstinspires.ftc.teamcode.hardware.Delivery;
import org.firstinspires.ftc.teamcode.hardware.DriveTrain;
import org.firstinspires.ftc.teamcode.hardware.Hang;
import org.firstinspires.ftc.teamcode.hardware.Intake;
import org.firstinspires.ftc.teamcode.hardware.LimitSwitch;
import org.firstinspires.ftc.teamcode.hardware.Slides;
import org.firstinspires.ftc.teamcode.hardware.Webcam;
import org.firstinspires.ftc.teamcode.utils.CurrentOpmode;
import org.firstinspires.ftc.teamcode.utils.PixelColors;
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
    private Blinkin blinkin;
    private Webcam webcam;
    private Intake intake;
    private Slides slides;
    private Claw claw;
    private Hang hang;
    //private BackboardDetector backboardDetector;

    private IntakeProcessor processor;

    private RobotState robotState;
    private boolean toBackboard;
    private int driveDeliveryTransition;

    private ElapsedTime transitionTimer, matchTimer;
    private int targetBackboardLevel = 0;
    private boolean cameraDisabled, backboardDropoffToggle, tiltToggle;

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
        blinkin = new Blinkin(hardwareMap);
        processor = new IntakeProcessor();
        webcam = new Webcam(hardwareMap, processor, "intake webcam");
        intake = new Intake(hardwareMap);
        slides = new Slides(hardwareMap);
        claw = new Claw(hardwareMap);
        hang = new Hang(hardwareMap);
        //backboardDetector = new BackboardDetector(hardwareMap);

        transitionTimer = new ElapsedTime();
        transitionTimer.startTime();

        matchTimer = new ElapsedTime();
        matchTimer.startTime();

        robotState = RobotState.DRIVE;

        telemetry.addLine("Ready to start");
        delivery.setPixelOrientation(Delivery.PixelOrientation.NORMAL);
    }

    @Override
    public void start() {
        intake.setServoPosition(Intake.IntakeState.STACK_MID);
    }

    @Override
    public void loop() {
        update();

        switch (robotState) {
            // Drive -> Intake
            case DRIVE_INTAKE_TRANSITION:
                delivery.setDeliveryState(Delivery.DeliveryState.INTAKE_HOLD);
                delivery.setPixelOrientation(Delivery.PixelOrientation.NORMAL);
                slides.setHeight(Slides.SlidesHeights.BASE);
                claw.setClawState(Claw.ClawState.OPEN_INTAKE);
                intake.setServoPosition(Intake.IntakeState.GROUND);
                intake.on();

                transitionTimer.reset();
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
                    intake.setServoPosition(Intake.IntakeState.STACK_HIGH);
                } else if (controller1.fallingEdgeOf(GamepadEx.Buttons.SQUARE)) {
                    intake.setServoPosition(Intake.IntakeState.GROUND);
                }

                if (!cameraDisabled && processor.hasTwoPixel()) {
                    gamepad1.rumble(250);
                    if (transitionTimer.milliseconds() > 750) {
                        driveDeliveryTransition = 0;
                        robotState = RobotState.INTAKE_DRIVE_TRANSITION;
                    }

                    blinkin.setTwoPixel(PixelColors.NONE, PixelColors.NONE);
                } else if (!cameraDisabled && processor.hasOnePixel()) {
                    blinkin.setOnePixel(PixelColors.NONE);
                } else if (blinkin.getCurrentState() != Blinkin.CurrentState.NONE) {
                    blinkin.clear();
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
                        if (transitionTimer.milliseconds() > 750) {
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
                        if (transitionTimer.milliseconds() > 700) {
                            driveDeliveryTransition++;
                        }

                        break;
                    case 6:
                        // Move to safe transition point that works for both intake and score
                        delivery.setDeliveryState(Delivery.DeliveryState.TRANSITION_1);
                        driveDeliveryTransition++;
                        transitionTimer.reset();
                        break;
                    case 7:
                        // Wait <milliseconds> so the physical servo has time to actually move
                        if (transitionTimer.milliseconds() > 500) {
                            driveDeliveryTransition++;
                        }

                        break;
                    case 8:
                        // Move to safe transition point that works for both intake and score
                        delivery.setDeliveryState(Delivery.DeliveryState.TRANSITION_2);
                        driveDeliveryTransition++;
                        transitionTimer.reset();
                        break;
                    case 9:
                        // Advance
                        robotState = RobotState.DRIVE;
                        break;
                }

                if (controller1.risingEdgeOf(GamepadEx.Buttons.TRIANGLE)) {
                    robotState = RobotState.DRIVE_INTAKE_TRANSITION;
                }

                if (controller1.risingEdgeOf(GamepadEx.Buttons.BUMPER_RIGHT)) {
                    intake.reverse();
                } else if (controller1.fallingEdgeOf(GamepadEx.Buttons.BUMPER_RIGHT)) {
                    intake.off();
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

                if (controller1.risingEdgeOf(GamepadEx.Buttons.BUMPER_RIGHT)) {
                    intake.reverse();
                } else if (controller1.fallingEdgeOf(GamepadEx.Buttons.BUMPER_RIGHT)) {
                    intake.off();
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
                        if (slides.atTargetPosition() || transitionTimer.milliseconds() > 750) {
                            driveDeliveryTransition++;
                        }

                        break;
                    case 2:
                        blinkin.strobe();
                        if (targetBackboardLevel < 2) {
                            delivery.setDeliveryState(Delivery.DeliveryState.SCORE);
                        } else {
                            delivery.setDeliveryState(Delivery.DeliveryState.SCORE_NO_OUT);
                        }
                        //backboardDetector.clear();

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
                    if (targetBackboardLevel < 4) {
                        targetBackboardLevel++;

                        if (targetBackboardLevel > 2) {
                            delivery.setDeliveryState(Delivery.DeliveryState.SCORE_NO_OUT);
                        }

                        slides.setHeight(Slides.SlidesHeights.levelFromInt(targetBackboardLevel));
                    }
                }

                if (controller1.risingEdgeOf(GamepadEx.Buttons.BUMPER_LEFT)) {
                    if (targetBackboardLevel > 0) {
                        targetBackboardLevel--;

                        if (targetBackboardLevel < 2) {
                            delivery.setDeliveryState(Delivery.DeliveryState.SCORE);
                        }

                        slides.setHeight(Slides.SlidesHeights.levelFromInt(targetBackboardLevel));
                    }
                }

                if (controller1.leftTriggerRisingEdge()) {
                    if (!tiltToggle) {
                        delivery.setPixelOrientation(Delivery.PixelOrientation.LEFT_45);
                    } else {
                        delivery.setPixelOrientation(Delivery.PixelOrientation.NORMAL);
                    }
                    tiltToggle = !tiltToggle;
                    backboardDropoffToggle = !backboardDropoffToggle;
                }

                if (controller1.rightTriggerRisingEdge()) {
                    if (!tiltToggle) {
                        delivery.setPixelOrientation(Delivery.PixelOrientation.RIGHT_45);
                    } else {
                        delivery.setPixelOrientation(Delivery.PixelOrientation.NORMAL);
                    }
                    tiltToggle = !tiltToggle;
                    backboardDropoffToggle = !backboardDropoffToggle;
                }

                if (controller1.risingEdgeOf(GamepadEx.Buttons.CROSS)) {
                    driveDeliveryTransition = 0;
                    robotState = RobotState.SCORE_DRIVE_TRANSITION;
                }

                /*
                if (!backboardDropoffToggle && backboardDetector.isReady() && backboardDetector.getAngle() < 15) {
                    if (backboardDetector.getAverageDistance() < 10) {
                        driveDeliveryTransition = 0;
                        robotState = RobotState.SCORE_DRIVE_TRANSITION;
                    }
                }
                 */

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
                        delivery.setDeliveryState(Delivery.DeliveryState.TRANSITION_1);
                        blinkin.clear();

                        transitionTimer.reset();
                        driveDeliveryTransition++;
                        break;
                    case 3:
                        if (transitionTimer.milliseconds() > 1000) {
                            driveDeliveryTransition++;
                        }

                        break;
                    case 4:
                        // Move to transition point between intake and score
                        slides.setHeight(Slides.SlidesHeights.BASE);
                        delivery.setDeliveryState(Delivery.DeliveryState.TRANSITION_1);
                        delivery.setPixelOrientation(Delivery.PixelOrientation.NORMAL);
                        tiltToggle = false;
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
                blinkin.setEndgame();

                if (transitionTimer.milliseconds() > 250) {
                    robotState = RobotState.ENDGAME;
                }

                break;
            case ENDGAME:
                hang.motor(gamepad1.left_trigger - gamepad1.right_trigger);

                if (controller1.risingEdgeOf(GamepadEx.Buttons.DPAD_DOWN)){
                    hang.setHangState(Hang.HangState.DOWN);
                }

                if (controller1.risingEdgeOf(GamepadEx.Buttons.SQUARE)) {
                    airplane.launch();
                }

                if (controller1.risingEdgeOf(GamepadEx.Buttons.OPTION)) {
                    hang.setHangState(Hang.HangState.DOWN);
                    robotState = RobotState.DRIVE;
                }

                break;
        }

        if (controller1.risingEdgeOf(GamepadEx.Buttons.DPAD_UP)) {
            robotState = RobotState.TRANSITION_ENDGAME;
            transitionTimer.reset();
        }

        if (controller1.risingEdgeOf(GamepadEx.Buttons.CIRCLE)) {
            if (targetBackboardLevel < 4) {
                targetBackboardLevel++;
            }
        }

        if (controller2.risingEdgeOf(GamepadEx.Buttons.TRIANGLE)) {
            slides.setPidEnabled(false);
            slides.manual(-0.2);
        } else if (controller2.fallingEdgeOf(GamepadEx.Buttons.TRIANGLE)) {
            slides.manual(0);
            slides.resetEncoders();
            slides.setPidEnabled(true);
        }

        if (slideLimit.isRisingEdge()) {
            slides.resetEncoders();
        }

        if (controller2.risingEdgeOf(GamepadEx.Buttons.SQUARE)) {
            cameraDisabled = !cameraDisabled;
        }

        if (controller2.risingEdgeOf(GamepadEx.Buttons.CIRCLE)) {
            backboardDropoffToggle = true;
        }

        write();
    }

    private void update() {
        // Need to be the first things
        controller1.update();
        controller2.update();

        slideLimit.update();
        slides.update();

        if (!cameraDisabled && (robotState == RobotState.DRIVE_INTAKE_TRANSITION || robotState == RobotState.INTAKE)) {
            processor.update();
        }

        if (robotState == RobotState.SCORE) {
            delivery.update();
            //backboardDetector.update();
        }
    }

    private void write() {
        driveTrain.writeTeleOp();

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
        telemetry.addData("Left Intake Pixel", processor.getLeftPixel());
        telemetry.addData("Right Intake Pixel", processor.getRightColor());
        telemetry.addData("Camera enabled", !cameraDisabled);
        /*
        telemetry.addData("Angle of Robot", backboardDetector.getAngle());
        telemetry.addData("Distance of robot", backboardDetector.getAverageDistance());
        telemetry.addData("Left distance", backboardDetector.getLeft());
        telemetry.addData("Right distance", backboardDetector.getRight());
        telemetry.addData("Auto-dropoff ready", backboardDetector.isReady());
         */
        telemetry.addData("Robot", robotState);

        telemetry.addData("Loop time", matchTimer.milliseconds());
        matchTimer.reset();
    }
}
