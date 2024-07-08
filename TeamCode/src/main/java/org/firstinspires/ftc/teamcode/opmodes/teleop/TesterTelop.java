package org.firstinspires.ftc.teamcode.opmodes.teleop;

import android.net.wifi.aware.IdentityChangedListener;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.hardware.Delivery;
import org.firstinspires.ftc.teamcode.hardware.Slides;
import org.firstinspires.ftc.teamcode.hardware.StackColorSensor;
import org.firstinspires.ftc.teamcode.hardware.Webcam;
import org.firstinspires.ftc.teamcode.utils.CurrentOpmode;
import org.firstinspires.ftc.teamcode.vision.IntakeProcessor;

@Config
@TeleOp()
public class TesterTelop extends OpMode {
    public static double target = 0;

    private Webcam webcam;
    private IntakeProcessor processor;


    @Override
    public void init() {
        processor = new IntakeProcessor();
        webcam = new Webcam(hardwareMap, processor, "intake webcam");
    }

    @Override
    public void loop() {
        telemetry.addData("left pixel ", processor.getLeftPixel());
        telemetry.addData("right pixel ", processor.getRightPixel());
        telemetry.addData("right pixel mean 0", processor.hasTwoPixel());
        telemetry.addData("right pixel mean 1", processor.hasOnePixel());
        telemetry.update();
    }
}