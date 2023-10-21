package org.firstinspires.ftc.teamcode.vision;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

public class PropProcessor implements VisionProcessor {
    public enum Spikes {
        LEFT,
        RIGHT,
        CENTER,
        UNKNOWN
    }

    private static final Rect LEFT_SPIKE = new Rect(0, 10, 426, 700);
    private static final Rect MIDDLE_SPIKE = new Rect(0, 10, 426, 700);
    private static final Rect RIGHT_SPIKE = new Rect(0, 10, 426, 700);

    private static final Size BLUR_SIZE = new Size(5, 5);

    private double[] spikeLocations = new double[2];
    private int largestSpikeIndex = -1;

    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Imgproc.cvtColor(frame, frame, Imgproc.COLOR_RGB2HSV);
        Imgproc.GaussianBlur(frame, frame, BLUR_SIZE, 0);

        spikeLocations[0] = Core.mean(frame.submat(LEFT_SPIKE)).val[0];
        spikeLocations[1] = Core.mean(frame.submat(MIDDLE_SPIKE)).val[0];
        spikeLocations[2] = Core.mean(frame.submat(RIGHT_SPIKE)).val[0];

        for (int i = 0; i < 3; i++) {
            if (spikeLocations[i] > largestSpikeIndex) {
                largestSpikeIndex = i;
            }
        }

        return frame;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }

    public Spikes getSpikePosition() {
        switch (largestSpikeIndex) {
            case 0:     return Spikes.LEFT;
            case 1:     return Spikes.CENTER;
            case 2:     return Spikes.RIGHT;
            default:    return Spikes.UNKNOWN;
        }
    }
}
