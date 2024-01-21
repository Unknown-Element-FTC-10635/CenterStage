package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.utils.PixelColors;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.logging.Logger;

public class IntakeProcessor extends SimpleProcessor {
    private final Logger logger = Logger.getLogger(this.getClass().getName());

    private static final Rect LEFT_INTAKE = new Rect(390, 390, 90, 60);
    private static final Rect RIGHT_INTAKE = new Rect(690, 390, 90, 60);

    private static final Size BLUR_SIZE = new Size(5, 5);

    private final Mat processMat = new Mat();

    private Scalar leftMean, rightMean;

    public IntakeProcessor() {
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        logger.info("starting!");
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Imgproc.GaussianBlur(frame, processMat, BLUR_SIZE, 0);

        leftMean = Core.mean(processMat.submat(LEFT_INTAKE));
        rightMean = Core.mean(processMat.submat(RIGHT_INTAKE));

        Imgproc.rectangle(frame, LEFT_INTAKE, leftMean, 5);
        Imgproc.rectangle(frame, RIGHT_INTAKE, rightMean, 5);

        processMat.release();
        super.processFrame(frame, captureTimeNanos);
        return frame;
    }

    public PixelColors getLeftPixel() {
        if (leftMean != null) {
            return getPixel(leftMean.val);
        }

        return PixelColors.NONE;
    }

    public PixelColors getRightColor() {
        if (rightMean != null) {
            return getPixel(rightMean.val);
        }

        return PixelColors.NONE;
    }

    public Scalar getLeftMean() {
        return leftMean;
    }

    public Scalar getRightMean() {
        return rightMean;
    }

    private PixelColors getPixel(double[] val) {
        if (val.length != 4) {
            return PixelColors.NONE;
        }


        if (val[0] >= 190 && val[1] >= 190 && val[2] >= 190) {
            return PixelColors.WHITE;
        } else if (val[1] >= 140 && val[0] < val[1] && val[2] < val[1]) {
            return PixelColors.GREEN;
        } else if (val[2] >= 150 && val[0] < val[2] && val[1] < val[2]) {
            return PixelColors.PURPLE;
        } else if (val[0] >= 175 && val[1] < val[0] && val[2] < val[0]) {
            return PixelColors.YELLOW;
        }

        return PixelColors.NONE;
    }
}
