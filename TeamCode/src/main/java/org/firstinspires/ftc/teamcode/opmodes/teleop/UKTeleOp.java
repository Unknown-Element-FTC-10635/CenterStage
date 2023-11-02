package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Airplane;
import org.firstinspires.ftc.teamcode.hardware.Claw;
import org.firstinspires.ftc.teamcode.hardware.Delivery;
import org.firstinspires.ftc.teamcode.hardware.DriveTrain;
import org.firstinspires.ftc.teamcode.hardware.Intake;
import org.firstinspires.ftc.teamcode.utils.CurrentOpmode;
import org.firstinspires.ftc.teamcode.utils.hardware.GamepadEx;

@TeleOp
public class UKTeleOp extends OpMode {
    public enum RobotState {
        INTAKE_TRANSITION,
        INTAKE,
        DRIVE_TRANSITION,
        DRIVE,
        SCORE_TRANSITION,
        SCORE
    }

    private GamepadEx controller1, controller2;
    private DriveTrain driveTrain;
    private Airplane airplane;
    private Delivery delivery;
    private Intake intake;
    private Claw claw;

    private RobotState robotState;
    private boolean toBackboard;

    @Override
    public void init() {
        CurrentOpmode.setCurrentOpmode(CurrentOpmode.OpMode.TELEOP);

        controller1 = new GamepadEx(gamepad1);
        controller2 = new GamepadEx(gamepad2);

        driveTrain = new DriveTrain(hardwareMap, controller1);
        airplane = new Airplane(hardwareMap);
        delivery = new Delivery(hardwareMap);
        intake = new Intake(hardwareMap);
        claw = new Claw(hardwareMap);

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
            case INTAKE_TRANSITION:
                delivery.setDeliveryState(Delivery.DeliveryState.INTAKE);
                claw.setClawState(Claw.ClawState.OPEN_INTAKE);
                intake.on();

                toBackboard = true;
                robotState = RobotState.INTAKE;
                break;
            case INTAKE:
                if (controller1.risingEdgeOf(GamepadEx.Buttons.L3)) {
                    robotState = RobotState.DRIVE_TRANSITION;
                }

                break;
            case DRIVE_TRANSITION:
                claw.setClawState(Claw.ClawState.CLOSED);
                intake.off();

                robotState = RobotState.DRIVE;
                break;
            case DRIVE:
                if (controller1.risingEdgeOf(GamepadEx.Buttons.L3)) {
                    if (toBackboard) {
                        robotState = RobotState.SCORE_TRANSITION;
                    } else {
                        robotState = RobotState.INTAKE_TRANSITION;
                    }
                }

                break;
            case SCORE_TRANSITION:
                toBackboard = false;
                robotState = RobotState.SCORE;
                break;
            case SCORE:
                if (controller1.risingEdgeOf(GamepadEx.Buttons.CROSS)) {
                    claw.setClawState(Claw.ClawState.OPEN_SCORE);
                }

                if (controller1.risingEdgeOf(GamepadEx.Buttons.L3)) {
                    robotState = RobotState.DRIVE_TRANSITION;
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

        delivery.update();
    }

    private void write() {
        driveTrain.writeTeleOp();

        telemetry.addData("Speed", driveTrain.getSpeedMultiplier());
        telemetry.addData("Delivery", delivery.getDeliveryState());
        telemetry.addData("Robot", robotState);
    }
}
