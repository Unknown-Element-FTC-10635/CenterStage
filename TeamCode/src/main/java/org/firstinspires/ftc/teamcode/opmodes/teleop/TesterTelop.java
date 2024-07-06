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


        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addLine("Ready to start");
        telemetry.update();


        waitForStart();

        while (opModeIsActive()) {
            delivery.setDeliveryState(Delivery.DeliveryState.INTAKE_PICKUP);
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
