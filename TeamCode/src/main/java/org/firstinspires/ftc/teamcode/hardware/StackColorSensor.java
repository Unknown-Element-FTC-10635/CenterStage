package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class StackColorSensor {
    public static final double LEFT_SENSOR_OFFSET_CM = -5;
    public static final double RIGHT_SENSOR_OFFSET_CM = 5;

    private final RevColorSensorV3 colorSensor;
    private final boolean blue;

    private double rawLight, distance;

    public StackColorSensor(HardwareMap hardwareMap, boolean blue) {
        this.blue = blue;
        if (blue) {
            this.colorSensor = hardwareMap.get(RevColorSensorV3.class, "left color");
        } else {
            this.colorSensor = hardwareMap.get(RevColorSensorV3.class, "right color");
        }
    }

    public void update() {
        rawLight = colorSensor.getRawLightDetected();
        distance = colorSensor.getDistance(DistanceUnit.CM);
    }

    public boolean linedUpWithStack() {
        if (blue) {
            return rawLight > 100;
        } else {
            return rawLight > 115;
        }
    }

    public boolean correctDistanceFromStack() {
        if (blue) {
            return distance < 5;
        } else {
            return distance < 4;
        }
    }

    public double getRawLight() {
        return rawLight;
    }

    public double getDistance() {
        return distance;
    }
}
