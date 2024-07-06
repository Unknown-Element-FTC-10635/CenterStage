package org.firstinspires.ftc.teamcode.opmodes.teleop;

import android.net.wifi.aware.IdentityChangedListener;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.hardware.Delivery;
import org.firstinspires.ftc.teamcode.hardware.Slides;
import org.firstinspires.ftc.teamcode.hardware.StackColorSensor;
import org.firstinspires.ftc.teamcode.utils.CurrentOpmode;

@Config
@TeleOp()
public class TesterTelop extends LinearOpMode {
    public static double target = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        CurrentOpmode.setCurrentOpmode(CurrentOpmode.OpMode.TELEOP);

        StackColorSensor leftColorSensor = new StackColorSensor(hardwareMap, true);
        StackColorSensor rightColorSensor = new StackColorSensor(hardwareMap, false);
        Delivery delivery = new Delivery(hardwareMap);
        Slides slides = new Slides(hardwareMap);


        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addLine("Ready to start");
        telemetry.update();


        waitForStart();

        while (opModeIsActive()) {
            delivery.setDeliveryState(Delivery.DeliveryState.SCORE);
            slides.setHeight(Slides.SlidesHeights.TENTH_LEVEL);


            telemetry.addData("Raw light", slides.atTargetPosition());
            telemetry.addData("left pos", slides.getCurrentLeftPosition());
            telemetry.addData("left", slides.getCurrentLeftPower());
            telemetry.addData("right", slides.getCurrentRightPower());
            telemetry.addData("right pos", slides.getCurrentRightPosition());


            telemetry.update();
        }
    }
}
