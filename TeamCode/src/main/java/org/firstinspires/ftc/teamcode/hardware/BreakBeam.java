package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class BreakBeam {
    private final DigitalChannel breakBeam;

    private boolean state;

    public BreakBeam(HardwareMap hardwareMap, String name) {
        breakBeam = hardwareMap.get(DigitalChannel.class, name);
    }

    public boolean broken() {
        return !state;
    }

    public void update() {
        state = breakBeam.getState();
    }
}
