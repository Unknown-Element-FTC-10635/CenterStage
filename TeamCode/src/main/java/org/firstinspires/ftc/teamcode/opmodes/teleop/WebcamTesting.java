package org.firstinspires.ftc.teamcode.opmodes.teleop;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.hardware.DriveTrain;
import org.firstinspires.ftc.teamcode.hardware.Webcam;
import org.firstinspires.ftc.teamcode.utils.CurrentOpmode;
import org.firstinspires.ftc.teamcode.utils.hardware.GamepadEx;
import org.firstinspires.ftc.teamcode.vision.IntakeProcessor;
import org.firstinspires.ftc.teamcode.vision.PropProcessor;
import org.firstinspires.ftc.vision.VisionProcessor;

@TeleOp()
public class WebcamTesting extends LinearOpMode {
    @Overrid
    public void runOpMode() throws InterruptedException {
        CurrentOpmode.setCurrentOpmode(CurrentOpmode.OpMode.TELEOP);

        PropProcessor processor = new PropProcessor(false);
        Webcam webcam = new Webcam(hardwareMap, processor, "webcam");

        telemetry.addLine("Ready to start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Camera FPS", webcam.getFPS());
            telemetry.addData("Spike Position", processor.getSpikePosition());
            telemetry.update();
        }
    }
}