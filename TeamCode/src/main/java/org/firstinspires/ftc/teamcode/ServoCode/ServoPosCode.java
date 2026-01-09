package org.firstinspires.ftc.teamcode.ServoCode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ServoPosCode {
    private Servo servoPos;
   // private CRServo servoCon;

    public void init(HardwareMap hwMap) {
        servoPos = hwMap.get(Servo.class, "servo_pos");
//        servoCon = hwMap.get(CRServo.class, "servo_con");
    }

    public void setServoPos(double angle)    {
        servoPos.setPosition(angle);
    }
  /*  public void setServoCon(double speed) {
        servoCon.setPower(speed);
    }*/
}
