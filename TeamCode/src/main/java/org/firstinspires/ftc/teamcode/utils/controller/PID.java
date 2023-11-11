package org.firstinspires.ftc.teamcode.utils.controller;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.opencv.core.Mat;

public class PID {
    private static final double MAX_INTEGRAL = 1.0;
    private static final double MIN_INTEGRAL = -1.0;

    private final ElapsedTime elapsedTime;
    private final double kP, kI, kD;

    private double setPoint, processVariable;
    private double error, prevError, totalError, time, prevTime;

    public PID(double setPoint, double processVariable, double kP, double kI, double kD) {
        this.setPoint = setPoint;
        this.processVariable = processVariable;

        this.kP = kP;
        this.kI = kI;
        this.kD = kD;

        elapsedTime = new ElapsedTime();
        elapsedTime.startTime();
    }

    public double update(double pV) {
        this.processVariable = pV;

        prevError = error;
        error = setPoint - processVariable;

        totalError += error;
        if (totalError > MAX_INTEGRAL) {
            totalError = MAX_INTEGRAL;
        } else if (totalError < MIN_INTEGRAL) {
            totalError = MIN_INTEGRAL;
        }

        prevTime = time;
        time = elapsedTime.milliseconds();

        return calculateProportional() + calculateIntegral() + calculateDerivative();
    }

    public void setSetPoint(double setPoint) {
        this.setPoint = setPoint;
    }

    private double calculateProportional() {
        return kP * error;
    }

    private double calculateIntegral() {
        return kI * totalError;
    }

    private double calculateDerivative() {
        double dt = time - prevTime;
        return kD * ((error - prevError) / dt);
    }
}
