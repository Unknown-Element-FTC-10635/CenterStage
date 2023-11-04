package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LimitSwitch {
    private final DigitalChannel limitSwitch;

    private boolean state;
    private boolean isInverted = false;

    public LimitSwitch(HardwareMap hardwareMap, String name) {
        this.limitSwitch = hardwareMap.get(DigitalChannel.class, name);
    }

    public boolean isPressed() {
        return state;
    }

    public void setInverted(boolean invert) {
        isInverted = invert;
    }

    public void update() {
        state = isInverted ? !limitSwitch.getState() : limitSwitch.getState();
    }
}
