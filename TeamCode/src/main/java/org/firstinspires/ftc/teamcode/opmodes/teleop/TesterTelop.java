package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.BreakBeam;
import org.firstinspires.ftc.teamcode.hardware.Claw;
import org.firstinspires.ftc.teamcode.hardware.Delivery;
import org.firstinspires.ftc.teamcode.hardware.DriveTrain;
import org.firstinspires.ftc.teamcode.hardware.Intake;
import org.firstinspires.ftc.teamcode.hardware.Slides;
import org.firstinspires.ftc.teamcode.utils.CurrentOpmode;
import org.firstinspires.ftc.teamcode.utils.hardware.GamepadEx;

@TeleOp()
public class TesterTelop extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        CurrentOpmode.setCurrentOpmode(CurrentOpmode.OpMode.TELEOP);

        GamepadEx controller = new GamepadEx(gamepad1);
        DriveTrain driveTrain = new DriveTrain(hardwareMap, controller);
        Delivery delivery = new Delivery(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Slides slides = new Slides(hardwareMap);
        Claw claw = new Claw(hardwareMap);
        BreakBeam leftBeam = new BreakBeam(hardwareMap, "left break");

        telemetry.addLine("Ready to start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            controller.update();
            leftBeam.update();

            slides.manual(gamepad1.right_trigger - gamepad1.left_trigger);

            if (controller.risingEdgeOf(GamepadEx.Buttons.TRIANGLE)) {
                delivery.setDeliveryState(Delivery.DeliveryState.INTAKE_PICKUP);
            }

            if (controller.risingEdgeOf(GamepadEx.Buttons.SQUARE)) {
                delivery.setDeliveryState(Delivery.DeliveryState.INTAKE_HOLD);
            }

            if (controller.risingEdgeOf(GamepadEx.Buttons.CROSS)) {
                claw.setClawState(Claw.ClawState.CLOSED);
            }

            if (controller.risingEdgeOf(GamepadEx.Buttons.CIRCLE)) {
                claw.setClawState(Claw.ClawState.OPEN_INTAKE);
            }

            telemetry.addData("Claw", claw.getClawState());
            telemetry.addData("Left", slides.getCurrentLeftPosition());
            telemetry.addData("Right", slides.getCurrentRightPosition());
            telemetry.addData("Left Bream", leftBeam.broken());
            telemetry.addData("Left Front", driveTrain.getEncoderValues()[0]);
            telemetry.addData("Right Front", driveTrain.getEncoderValues()[1]);
            telemetry.addData("Left Back", driveTrain.getEncoderValues()[2]);
            telemetry.addData("Right Back", driveTrain.getEncoderValues()[3]);
            telemetry.update();
        }
    }
}
