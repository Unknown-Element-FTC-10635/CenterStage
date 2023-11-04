package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Claw;
import org.firstinspires.ftc.teamcode.hardware.DriveTrain;
import org.firstinspires.ftc.teamcode.hardware.Webcam;
import org.firstinspires.ftc.teamcode.utils.CurrentOpmode;
import org.firstinspires.ftc.teamcode.utils.hardware.GamepadEx;

@TeleOp()
public class TesterTelop extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        CurrentOpmode.setCurrentOpmode(CurrentOpmode.OpMode.TELEOP);

        GamepadEx controller = new GamepadEx(gamepad1);
        Claw claw = new Claw(hardwareMap);

        telemetry.addLine("Ready to start");
        telemetry.update();

        waitForStart();


        while (opModeIsActive()) {
            if (controller.risingEdgeOf(GamepadEx.Buttons.CROSS)) {
                claw.setClawState(Claw.ClawState.CLOSED);
            }

            if (controller.risingEdgeOf(GamepadEx.Buttons.CIRCLE)) {
                claw.setClawState(Claw.ClawState.OPEN_SCORE);
            }

            if (controller.risingEdgeOf(GamepadEx.Buttons.SQUARE)) {
                claw.setClawState(Claw.ClawState.OPEN_INTAKE);
            }

            if (controller.risingEdgeOf(GamepadEx.Buttons.TRIANGLE)) {
                claw.setClawState(Claw.ClawState.SUPER_CLOSE);
            }

            telemetry.update();
        }
    }
}
