package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class BackboardDetector {
    private static final double DISTANCE_BETWEEN_SENSORS = 5.0;
    private static final double C = Math.pow(DISTANCE_BETWEEN_SENSORS, 2);

    private final Rev2mDistanceSensor leftDistance, rightDistance;

    private double left, right, angle;

    public BackboardDetector(HardwareMap hardwareMap) {
        leftDistance = hardwareMap.get(Rev2mDistanceSensor.class, "distance left");
        rightDistance = hardwareMap.get(Rev2mDistanceSensor.class, "distance left");
    }

    public void update() {
        left = leftDistance.getDistance(DistanceUnit.CM);
        right = rightDistance.getDistance(DistanceUnit.CM);

        double a = Math.max(left, right);
        double b = Math.min(left, right);
        double d = Math.sqrt(Math.pow(a - b, 2) + C);

        angle = Math.toDegrees(Math.asin(d / (a-b)));
    }

    public double getLeft() {
        return left;
    }

    public double getRight() {
        return right;
    }

    public double getAngle() {
        return angle;
    }
}
