package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.StackColorSensor;
import org.firstinspires.ftc.teamcode.utils.CurrentOpmode;

@Config
@Disabled
@TeleOp()
public class TesterTelop extends LinearOpMode {
    public static double target = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        CurrentOpmode.setCurrentOpmode(CurrentOpmode.OpMode.TELEOP);

        StackColorSensor leftColorSensor = new StackColorSensor(hardwareMap, true);
        StackColorSensor rightColorSensor = new StackColorSensor(hardwareMap, false);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addLine("Ready to start");
        telemetry.update();


        waitForStart();

        while (opModeIsActive()) {
            leftColorSensor.update();
            rightColorSensor.update();

            telemetry.addData("Raw light", leftColorSensor.getRawLight());
            telemetry.addData("Distance", leftColorSensor.getDistance());
            telemetry.addData("Raw light", rightColorSensor.getRawLight());
            telemetry.addData("Distance", rightColorSensor.getDistance());
            telemetry.addData("target", target);
            telemetry.update();
        }
    }
}
