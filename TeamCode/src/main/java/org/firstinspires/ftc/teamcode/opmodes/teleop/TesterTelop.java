package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Robot;

@TeleOp()
public class TesterTelop extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Ready to start");
        telemetry.update();

        Robot robot = new Robot(hardwareMap);
        robot.init();

        waitForStart();

        robot.webcam.setCurrentProcessor(robot.webcam.aprilTagProcessor);

        while (opModeIsActive()) {
            robot.driveTrain.driveTeleOp(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

            telemetry.addData("Camera FPS", robot.webcam.getFPS());
            telemetry.update();
        }
    }
}
