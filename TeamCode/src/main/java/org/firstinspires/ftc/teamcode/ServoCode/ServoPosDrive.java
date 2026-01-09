package org.firstinspires.ftc.teamcode.ServoCode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ServoPosDrive {
    private Servo servoPos;

    public void ServoInit(HardwareMap hwMap) {
        servoPos = hwMap.get(Servo.class, "servo_pos");
    }

    public void setServoPos(double angle) {
        servoPos.setPosition(angle);
    }


}
