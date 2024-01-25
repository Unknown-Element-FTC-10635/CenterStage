/*       ________                   ________           ________          __________________     _____
        /        \                 /        \         |  _____ \         L______     _____|    |    |
       /   ____   \               /   ____   \        | |     | \              |    |          |    |
      /   /    \   \             /   /    \   \       | |      \ \             |    |          |    |
     /   /      \   \           /   /      \   \      | |       \ \            |    |          |    |
    /   /________\   \         /   /________\   \     | |        | |           |    |          |    |
   /   /__________\   \       /   /__________\   \    | |       / /            |    |          |    |
  /   /            \   \     /   /            \   \   | |      / /             |    |          |    |___________
 /   /              \   \   /   /              \   \  | |_____| /        ------     ------     |               |
/   /                \   \ /   /          `     \   \ |________/         L_______________|     |_______________| */

package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Airplane {
    private final Servo airplane;

    private static final double LAUNCH_POSITION = 0.3;

    public Airplane(HardwareMap hardwareMap) {
        airplane = hardwareMap.get(Servo.class, "airplane");
    }

    public void launch() {
        airplane.setPosition(LAUNCH_POSITION);
    }
}
