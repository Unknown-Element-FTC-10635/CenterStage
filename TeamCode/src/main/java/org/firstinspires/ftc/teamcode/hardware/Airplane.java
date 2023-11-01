package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Airplane {
    private final Servo airplane;

    private static final double HOLD_POSITION = 0.3;
    private static final double LAUNCH_POSITION = 0.0;

    public Airplane(HardwareMap hardwareMap) {
        airplane = hardwareMap.get(Servo.class, "airplane");
    }

    public void launch() {
       airplane.setPosition(LAUNCH_POSITION);
    }
}
