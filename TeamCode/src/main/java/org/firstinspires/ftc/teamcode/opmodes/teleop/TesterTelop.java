package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.hardware.Delivery;
import org.firstinspires.ftc.teamcode.hardware.Slides;
import org.firstinspires.ftc.teamcode.utils.CurrentOpmode;

@Config
@TeleOp()
@Disabled
public class TesterTelop extends LinearOpMode {
    public static double target = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        CurrentOpmode.setCurrentOpmode(CurrentOpmode.OpMode.TELEOP);

        Slides slides = new Slides(hardwareMap);
        Delivery delivery = new Delivery(hardwareMap);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addLine("Ready to start");
        telemetry.update();

        delivery.setDeliveryState(Delivery.DeliveryState.TRANSITION_2);

        waitForStart();

        while (opModeIsActive()) {
            slides.setHeight(target);

            slides.update();

            telemetry.addData("Left", slides.getCurrentLeftPosition());
            telemetry.addData("right", slides.getCurrentRightPosition());
            telemetry.addData("target", target);
            telemetry.update();
        }
    }
}
