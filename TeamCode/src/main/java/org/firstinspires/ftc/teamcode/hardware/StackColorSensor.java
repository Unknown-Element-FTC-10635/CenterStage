package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class StackColorSensor {
    public static final double LEFT_SENSOR_OFFSET_CM = -5;
    public static final double RIGHT_SENSOR_OFFSET_CM = 5;

    private final RevColorSensorV3 colorSensor;

    private double rawLight, distance;

    public StackColorSensor(HardwareMap hardwareMap, String name) {
        this.colorSensor = hardwareMap.get(RevColorSensorV3.class, name);
    }

    public void update() {
        rawLight = colorSensor.getRawLightDetected();
        distance = colorSensor.getDistance(DistanceUnit.CM);
    }

    public boolean linedUpWithStack() {
        return rawLight > 100;
    }

    public boolean correctDistanceFromStack() {
        return distance < 5;
    }

    public double getRawLight() {
        return rawLight;
    }

    public double getDistance() {
        return distance;
    }
}
