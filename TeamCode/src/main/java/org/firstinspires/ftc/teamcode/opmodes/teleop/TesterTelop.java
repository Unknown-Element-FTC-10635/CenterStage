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

        GamepadEx gamepad = new GamepadEx(gamepad1);
        Claw claw = new Claw(hardwareMap);
        Webcam webcam = new Webcam(hardwareMap);

        telemetry.addLine("Ready to start");
        telemetry.update();

        waitForStart();

        webcam.setCurrentProcessor(webcam.aprilTagProcessor);

        while (opModeIsActive()) {
            gamepad.update();

            if (gamepad.risingEdgeOf(GamepadEx.Buttons.CROSS)) {
                claw.setClawState(Claw.State.OPEN_SCORE);
         }

            if (gamepad.risingEdgeOf(GamepadEx.Buttons.SQUARE)) {
                claw.setClawState(Claw.State.CLOSED);
            }

            if (gamepad.risingEdgeOf(GamepadEx.Buttons.CIRCLE)) {
                claw.setClawState(Claw.State.OPEN_INTAKE);
            }

            if (gamepad.risingEdgeOf(GamepadEx.Buttons.TRIANGLE)) {
                claw.setClawState(Claw.State.SUPER_CLOSE);
            }

            telemetry.addData("Camera FPS", webcam.getFPS());
            telemetry.update();
        }
    }
}
