package org.firstinspires.ftc.teamcode.utils.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * A wrapper class for DcMotorEx that adds a builder pattern
 */
public class MotorBuilder {
    private final DcMotorEx motor;

    public MotorBuilder(HardwareMap hardwareMap, String motorName) {
        motor = hardwareMap.get(DcMotorEx.class, motorName);
    }

    public MotorBuilder setDirection(DcMotorSimple.Direction direction) {
        motor.setDirection(direction);
        return this;
    }

    public MotorBuilder setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        motor.setZeroPowerBehavior(behavior);
        return this;
    }

    public MotorBuilder resetEncoder() {
        DcMotor.RunMode currentMode = motor.getMode();
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(currentMode);
        return this;
    }

    public DcMotorEx build() {
        return motor;
    }
}
