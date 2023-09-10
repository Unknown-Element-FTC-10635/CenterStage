package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class Robot {
    public Webcam webcam;
    public DriveTrain driveTrain;

    private final HardwareMap hardwareMap;

    public Robot(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    public void init() {
        driveTrain = new DriveTrain(hardwareMap);
        webcam = new Webcam(hardwareMap);
    }
}
