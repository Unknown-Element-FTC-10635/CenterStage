package org.firstinspires.ftc.teamcode.hardware;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.vision.PropProcessor;
import org.firstinspires.ftc.teamcode.vision.SimpleProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class Webcam {
    public final SimpleProcessor simpleProcessor;
    public final AprilTagProcessor aprilTagProcessor;
    public final PropProcessor propProcessor;

    private final VisionPortal visionPortal;
    private final VisionProcessor[] visionProcessors;
    private VisionProcessor currentProcessor;

    public Webcam(HardwareMap hardwareMap) {
        simpleProcessor = new SimpleProcessor();
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .build();
        propProcessor = new PropProcessor();

        visionProcessors = new VisionProcessor[3];
        visionProcessors[0] = simpleProcessor;
        visionProcessors[1] = aprilTagProcessor;
        visionProcessors[2] = propProcessor;

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "webcam"))
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .enableLiveView(false)
                .addProcessors(simpleProcessor, aprilTagProcessor, propProcessor)
                .build();

        for (VisionProcessor processor : visionProcessors) {
            visionPortal.setProcessorEnabled(processor, false);
        }

        visionPortal.setProcessorEnabled(visionProcessors[0], true);
        currentProcessor = visionProcessors[0];
    }

    public void setCurrentProcessor(VisionProcessor processor) {
        visionPortal.setProcessorEnabled(currentProcessor, false);
        visionPortal.setProcessorEnabled(processor, true);
        currentProcessor = processor;
    }

    public void stopWebcam() {
        visionPortal.stopStreaming();
    }

    public float getFPS() {
        return visionPortal.getFps();
    }
}
