package org.firstinspires.ftc.teamcode.hardware;


import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.teamcode.vision.IntakeProcessor;
import org.firstinspires.ftc.teamcode.vision.PropProcessor;
import org.firstinspires.ftc.teamcode.vision.SimpleProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class Webcam {
    private final VisionPortal visionPortal;

    public Webcam(HardwareMap hardwareMap, SimpleProcessor processor, String name) {
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, name))
                .setCameraResolution(new Size(1280, 720))
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .enableLiveView(false)
                .addProcessors(processor)
                .build();

        FtcDashboard.getInstance().startCameraStream(processor, 0);
    }

    public void stopWebcam() {
        //visionPortal.stopStreaming();
    }

    public float getFPS() {
        return visionPortal.getFps();
    }
}
