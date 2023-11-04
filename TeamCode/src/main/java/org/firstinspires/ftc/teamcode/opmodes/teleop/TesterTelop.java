package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Claw;
import org.firstinspires.ftc.teamcode.hardware.Delivery;
import org.firstinspires.ftc.teamcode.utils.CurrentOpmode;
import org.firstinspires.ftc.teamcode.utils.hardware.GamepadEx;

@TeleOp()
public class TesterTelop extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        CurrentOpmode.setCurrentOpmode(CurrentOpmode.OpMode.TELEOP);

        GamepadEx controller = new GamepadEx(gamepad1);
        Delivery delivery = new Delivery(hardwareMap);
        Claw claw = new Claw(hardwareMap);

        telemetry.addLine("Ready to start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            controller.update();

            if (controller.risingEdgeOf(GamepadEx.Buttons.CROSS)) {
                claw.setClawState(Claw.ClawState.OPEN_INTAKE);
            }

            if (controller.risingEdgeOf(GamepadEx.Buttons.CIRCLE)) {
                claw.setClawState(Claw.ClawState.OPEN_SCORE);
            }

            if (controller.risingEdgeOf(GamepadEx.Buttons.SQUARE)) {
                claw.setClawState(Claw.ClawState.CLOSED);
            }

            telemetry.addData("Claw", claw.getClawState());
            telemetry.update();
        }
    }
}
