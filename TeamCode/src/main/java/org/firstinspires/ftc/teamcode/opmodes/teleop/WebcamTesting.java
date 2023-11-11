package org.firstinspires.ftc.teamcode.opmodes.teleop;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.hardware.DriveTrain;
import org.firstinspires.ftc.teamcode.hardware.Webcam;
import org.firstinspires.ftc.teamcode.utils.CurrentOpmode;
import org.firstinspires.ftc.teamcode.utils.hardware.GamepadEx;

@TeleOp()
public class WebcamTesting extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        CurrentOpmode.setCurrentOpmode(CurrentOpmode.OpMode.TELEOP);

        Webcam webcam = new Webcam(hardwareMap);

        telemetry.addLine("Ready to start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Pixel location", webcam.propProcessor.getSpikePosition());
            telemetry.addData("Camera FPS", webcam.getFPS());
            telemetry.update();
        }
    }
}