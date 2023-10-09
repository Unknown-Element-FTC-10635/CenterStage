package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.hardware.DriveTrain;
import org.firstinspires.ftc.teamcode.utils.CurrentOpmode;

public class UKTeleOp extends OpMode {
    private DriveTrain driveTrain;

    @Override
    public void init() {
        CurrentOpmode.setCurrentOpmode(CurrentOpmode.OpMode.TELEOP);

        driveTrain = new DriveTrain(hardwareMap);
    }

    @Override
    public void loop() {
        driveTrain.driveTeleOp(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
    }
}
