package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.hardware.DriveTrain;
import org.firstinspires.ftc.teamcode.hardware.Webcam;
import org.firstinspires.ftc.teamcode.utils.CurrentOpmode;
import org.firstinspires.ftc.teamcode.utils.hardware.GamepadEx;
import org.firstinspires.ftc.teamcode.vision.IntakeProcessor;
import org.firstinspires.ftc.vision.VisionProcessor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@TeleOp()
@Disabled
public class WebcamTesting extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        CurrentOpmode.setCurrentOpmode(CurrentOpmode.OpMode.TELEOP);

        IntakeProcessor processor = new IntakeProcessor();
        Webcam webcam = new Webcam(hardwareMap, processor, "intake webcam");

        telemetry.addLine("Ready to start");
        telemetry.update();

        waitForStart();

        webcam.stopWebcam();

        while (opModeIsActive()) {
            /*
            telemetry.addData("Camera FPS", webcam.getFPS());
            telemetry.addData("Left Pixel", processor.getLeftPixel());
            telemetry.addData("Left Mean", processor.getLeftMean());
            telemetry.addData("Right Pixel", processor.getRightColor());
            telemetry.addData("Right Mean", processor.getRightMean());
            */
            telemetry.update();
        }
    }
}