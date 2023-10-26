package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Plane {
    private Servo planeServo;

    public Plane(HardwareMap hardwareMap){
        planeServo = hardwareMap.get(Servo.class, "plane");
    }

    public void launch(){
        planeServo.setPosition(0.0);
    }


}
