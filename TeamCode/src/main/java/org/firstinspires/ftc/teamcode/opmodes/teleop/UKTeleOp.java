package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.DriveTrain;
import org.firstinspires.ftc.teamcode.utils.CurrentOpmode;
import org.firstinspires.ftc.teamcode.utils.hardware.GamepadEx;

@TeleOp()
public class UKTeleOp extends OpMode {
    private DriveTrain driveTrain;
    private GamepadEx controller1, controller2;

    @Override
    public void init() {
        CurrentOpmode.setCurrentOpmode(CurrentOpmode.OpMode.TELEOP);

        controller1 = new GamepadEx(gamepad1);
        controller2 = new GamepadEx(gamepad2);
        driveTrain = new DriveTrain(hardwareMap);
    }

    @Override
    public void loop() {
        update();

        if (controller1.risingEdgeOf(GamepadEx.Buttons.L3)) {
            driveTrain.toggleSpeedMultiplier();
        }

        write();
    }

    private void update() {
        // Need to be the first things
        controller1.update();
        controller2.update();

        driveTrain.updateTeleOp();
    }

    private void write() {

    }
}
