package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.DriveTrain;
import org.firstinspires.ftc.teamcode.hardware.Webcam;
import org.firstinspires.ftc.teamcode.utils.CurrentOpmode;

@TeleOp()
public class TesterTelop extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        CurrentOpmode.setCurrentOpmode(CurrentOpmode.OpMode.TELEOP);

        DriveTrain driveTrain = new DriveTrain(hardwareMap);
        Webcam webcam = new Webcam(hardwareMap);

        telemetry.addLine("Ready to start");
        telemetry.update();

        waitForStart();

        webcam.setCurrentProcessor(webcam.aprilTagProcessor);

        while (opModeIsActive()) {
            driveTrain.driveTeleOp(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

            telemetry.addData("Camera FPS", webcam.getFPS());
            telemetry.update();
        }
    }
}
