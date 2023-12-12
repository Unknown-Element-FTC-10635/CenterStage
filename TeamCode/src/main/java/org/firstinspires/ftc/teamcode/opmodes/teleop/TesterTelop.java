package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Claw;
import org.firstinspires.ftc.teamcode.hardware.Delivery;
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
        Intake intake = new Intake(hardwareMap);
        Slides slides = new Slides(hardwareMap);
        Claw claw = new Claw(hardwareMap);

        telemetry.addLine("Ready to start");
        telemetry.update();

        waitForStart();


        while (opModeIsActive()) {
            controller.update();

            slides.manual(gamepad1.right_trigger - gamepad1.left_trigger);

            if (controller.risingEdgeOf(GamepadEx.Buttons.TRIANGLE)) {
                intake.setServoPosition(Intake.IntakeState.GROUND);
            }

            if (controller.risingEdgeOf(GamepadEx.Buttons.SQUARE)) {
                intake.on();
            }

            if (controller.risingEdgeOf(GamepadEx.Buttons.CROSS)) {
                intake.setServoPosition(Intake.IntakeState.STACK_MID);
            }

            if (controller.risingEdgeOf(GamepadEx.Buttons.CIRCLE)) {
                intake.off();
                intake.setServoPosition(Intake.IntakeState.START_POSITION);
            }

            telemetry.addData("Claw", claw.getClawState());
            telemetry.addData("Left", slides.getCurrentLeftPosition());
            telemetry.addData("Right", slides.getCurrentRightPosition());
            telemetry.update();
        }
    }
}
