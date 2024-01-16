package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Blinkin;
import org.firstinspires.ftc.teamcode.utils.CurrentOpmode;
import org.firstinspires.ftc.teamcode.utils.PixelColors;
import org.firstinspires.ftc.teamcode.utils.hardware.GamepadEx;
import org.firstinspires.ftc.teamcode.vision.IntakeProcessor;

@TeleOp()
public class TesterTelop extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        CurrentOpmode.setCurrentOpmode(CurrentOpmode.OpMode.TELEOP);

        GamepadEx controller = new GamepadEx(gamepad1);
        Blinkin blinkin = new Blinkin(hardwareMap);
        blinkin.setOnePixel(PixelColors.GREEN);

        telemetry.addLine("Ready to start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            controller.update();

            if (controller.risingEdgeOf(GamepadEx.Buttons.SQUARE)) {
                blinkin.setOnePixel(PixelColors.GREEN);
            }

            if (controller.risingEdgeOf(GamepadEx.Buttons.CROSS)) {
                blinkin.setTwoPixel(PixelColors.GREEN, PixelColors.WHITE);
            }

            if (controller.risingEdgeOf(GamepadEx.Buttons.CIRCLE)) {
                blinkin.clear();
            }
        }
    }
}
