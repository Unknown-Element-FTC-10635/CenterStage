package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.hardware.MotorBuilder;

public class Hang {
    public enum HangState{
        UP(0.325, 0.325),
        DOWN(.9, .9);

        public final double rightServoPosition;
        public final double leftServoPosition;


        HangState(double rightServo, double leftServo){
            this.leftServoPosition = leftServo;
            this.rightServoPosition = rightServo;

        }
    }

    private final Servo rightServo;
    private final Servo leftServo;
    private final DcMotorEx hangMotor;

    private HangState hangState;

    public Hang(HardwareMap hardwareMap){
        rightServo = hardwareMap.get(Servo.class, "hang right");
        leftServo = hardwareMap.get(Servo.class, "hang left");
        leftServo.setDirection(Servo.Direction.REVERSE);
        hangMotor = new MotorBuilder(hardwareMap, "hang")
                .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
                .setDirection(DcMotorSimple.Direction.REVERSE)
                .build();

    }
    public void setHangState(HangState hangState){
        this.hangState = hangState;
        rightServo.setPosition(hangState.rightServoPosition);
        leftServo.setPosition(hangState.rightServoPosition);
    }

    public HangState getHangState(){
        return hangState;
    }

    public void motor(double power){
        hangMotor.setPower(power);
    }


}
