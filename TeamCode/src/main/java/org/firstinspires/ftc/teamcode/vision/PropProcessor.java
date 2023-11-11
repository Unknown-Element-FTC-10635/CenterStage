package org.firstinspires.ftc.teamcode.vision;

import android.graphics.Bitmap;
import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import java.util.concurrent.atomic.AtomicReference;

public class PropProcessor implements VisionProcessor, CameraStreamSource {
    private final AtomicReference<Bitmap> lastFrame =
            new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));


    @Override
    public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
        continuation.dispatch(bitmapConsumer -> bitmapConsumer.accept(lastFrame.get()));
    }

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

    private double[] spikeLocations = new double[3];
    private int largestSpikeIndex = -1;
    private Mat processMat= new Mat();

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        lastFrame.set(Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565));
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Imgproc.cvtColor(frame, processMat, Imgproc.COLOR_RGB2HSV);
        Imgproc.GaussianBlur(processMat, processMat, BLUR_SIZE, 0);


        spikeLocations[0] = Core.mean(processMat.submat(LEFT_SPIKE)).val[0];
        spikeLocations[1] = Core.mean(processMat.submat(MIDDLE_SPIKE)).val[0];
        spikeLocations[2] = Core.mean(processMat.submat(RIGHT_SPIKE)).val[0];

        Imgproc.rectangle(frame, LEFT_SPIKE, new Scalar(255, 0, 0), 5);
        Imgproc.rectangle(frame, RIGHT_SPIKE, new Scalar(255, 0, 0), 5);
        Imgproc.rectangle(frame, MIDDLE_SPIKE, new Scalar(255, 0, 0), 5);

        Bitmap b = Bitmap.createBitmap(processMat.width(), processMat.height(), Bitmap.Config.RGB_565);
        Utils.matToBitmap(frame, b);
        lastFrame.set(b);

        return frame;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }

    public Spikes getSpikePosition() {
        for (int i = 0; i < 1; i++) {
            if (spikeLocations[i] > largestSpikeIndex) {
                largestSpikeIndex = i;
            }
        }

        switch (largestSpikeIndex) {
            case 0:     return Spikes.LEFT;
            case 1:     return Spikes.CENTER;
            case 2:     return Spikes.RIGHT;
            default:    return Spikes.UNKNOWN;
        }
    }
}
