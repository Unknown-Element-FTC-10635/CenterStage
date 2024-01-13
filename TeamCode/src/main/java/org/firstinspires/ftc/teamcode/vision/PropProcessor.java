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
import java.util.logging.Logger;

public class PropProcessor implements VisionProcessor, CameraStreamSource {
    private final Logger logger = Logger.getLogger(this.getClass().getName());

    public enum Spikes {
        LEFT,
        RIGHT,
        CENTER,
        UNKNOWN
    }

    private static final Rect BLUE_LEFT_SPIKE = new Rect(50, 400, 200, 200);
    private static final Rect BLUE_MIDDLE_SPIKE = new Rect(500, 375, 200, 200);
    private static final Rect BLUE_RIGHT_SPIKE = new Rect(900, 440, 200, 200);

    private static final Rect RED_LEFT_SPIKE = new Rect(225, 430, 200, 200);
    private static final Rect RED_MIDDLE_SPIKE = new Rect(630, 375, 200, 200);
    private static final Rect RED_RIGHT_SPIKE = new Rect(1080, 440, 200, 200);

    private final Rect leftSpike, middleSpike, rightSpike;

    private static final Size BLUR_SIZE = new Size(5, 5);

    private final double[] spikeLocations = new double[3];
    private int largestSpikeIndex = 3;

    private final Mat processMat = new Mat();
    private final AtomicReference<Bitmap> lastFrame =
            new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));

    private final Scalar rectColor;

    public PropProcessor(boolean blue) {
        if (blue) {
            leftSpike = BLUE_LEFT_SPIKE;
            middleSpike = BLUE_MIDDLE_SPIKE;
            rightSpike = BLUE_RIGHT_SPIKE;
            rectColor = new Scalar(0, 0, 255);
        } else {
            leftSpike = RED_LEFT_SPIKE;
            middleSpike = RED_MIDDLE_SPIKE;
            rightSpike = RED_RIGHT_SPIKE;
            rectColor = new Scalar(255, 0, 0);
        }
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        logger.info("starting!");
        lastFrame.set(Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565));
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Imgproc.cvtColor(frame, processMat, Imgproc.COLOR_RGB2YCrCb);
        Imgproc.GaussianBlur(processMat, processMat, BLUR_SIZE, 0);

        spikeLocations[0] = Core.mean(processMat.submat(leftSpike)).val[0];
        spikeLocations[1] = Core.mean(processMat.submat(middleSpike)).val[0];
        spikeLocations[2] = Core.mean(processMat.submat(rightSpike)).val[0];

        Imgproc.rectangle(frame, leftSpike, rectColor, 5);
        Imgproc.rectangle(frame, middleSpike, rectColor, 5);
        Imgproc.rectangle(frame, rightSpike, rectColor, 5);

        Bitmap b = Bitmap.createBitmap(processMat.width(), processMat.height(), Bitmap.Config.RGB_565);
        Utils.matToBitmap(frame, b);
        lastFrame.set(b);

        return frame;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }

    public Spikes getSpikePosition() {
        largestSpikeIndex = 0;
        for (int i = 1; i < spikeLocations.length; i++) {
            if (spikeLocations[i] < spikeLocations[largestSpikeIndex]) {
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

    public double[] getValues(){
        return spikeLocations;
    }

    @Override
    public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
        continuation.dispatch(bitmapConsumer -> bitmapConsumer.accept(lastFrame.get()));
    }
}
