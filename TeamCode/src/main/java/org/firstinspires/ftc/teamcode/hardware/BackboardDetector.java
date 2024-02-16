package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class BackboardDetector {
    private static final double DISTANCE_BETWEEN_SENSORS = 10.16;
    private static final double LEFT_SENSOR_OFFSET = 0.3;

    private final Rev2mDistanceSensor leftDistance, rightDistance;

    private double[][] previousValues = new double[5][2];
    private double averageLeft, averageRight, averageDistance, averageAngle;
    private boolean ready;

    public BackboardDetector(HardwareMap hardwareMap) {
        leftDistance = hardwareMap.get(Rev2mDistanceSensor.class, "distance left");
        rightDistance = hardwareMap.get(Rev2mDistanceSensor.class, "distance right");
    }

    public void update() {
        double left = leftDistance.getDistance(DistanceUnit.CM);
        double right = rightDistance.getDistance(DistanceUnit.CM);

        double a = Math.max(left - LEFT_SENSOR_OFFSET, right);
        double b = Math.min(left - LEFT_SENSOR_OFFSET, right);

        shiftAndInsert(a, b);

        double[] tempDistance = averageABDistances();
        averageLeft = tempDistance[0];
        averageRight = tempDistance[1];

        averageAngle = Math.toDegrees(Math.atan((averageLeft - averageRight) / DISTANCE_BETWEEN_SENSORS));
        averageDistance = (tempDistance[0] + tempDistance[1]) / 2;
    }

    public void clear() {
        previousValues = new double[5][2];
        ready = false;
    }

    public boolean withinDropOfDistance(){
        return !Double.isNaN(averageDistance) && averageDistance < 10;
    }

    public boolean isReady() {
        return ready;
    }

    public double getLeft() {
        return averageLeft;
    }

    public double getRight() {
        return averageRight;
    }

    public double getAngle() {
        return averageAngle;
    }

    public double getAverageDistance() {
        return averageDistance;
    }

    private double getIdealDistance(){
        //TODO
        return 1.0;
    }

    private double[] averageABDistances() {
        double a = 0;
        double b = 0;

        for (double[] previousValue : previousValues) {
            a += previousValue[0];
            b += previousValue[1];
        }
        return new double[]{a / previousValues.length, b / previousValues.length};
    }

    private void shiftAndInsert(double a, double b) {
        previousValues[4] = previousValues[3];
        previousValues[3] = previousValues[2];
        previousValues[2] = previousValues[1];
        previousValues[1] = previousValues[0];
        previousValues[0] = new double[]{a, b};

        if (!ready && previousValues[4][0] != 0.0) {
            ready = true;
        }
    }
}
