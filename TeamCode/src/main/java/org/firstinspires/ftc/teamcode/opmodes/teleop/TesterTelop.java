package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.StackColorSensor;
import org.firstinspires.ftc.teamcode.utils.CurrentOpmode;

@Config
@TeleOp()
public class TesterTelop extends LinearOpMode {
    public static double target = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        CurrentOpmode.setCurrentOpmode(CurrentOpmode.OpMode.TELEOP);

        StackColorSensor colorSensor = new StackColorSensor(hardwareMap, "left color");

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addLine("Ready to start");
        telemetry.update();


        waitForStart();

        while (opModeIsActive()) {
            colorSensor.update();

            telemetry.addData("Raw light", colorSensor.getRawLight());
            telemetry.addData("Distance", colorSensor.getDistance());
            telemetry.addData("Stack guess", colorSensor.linedUpWithStack());
            telemetry.addData("target", target);
            telemetry.update();
        }
    }
}
