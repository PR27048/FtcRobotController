package org.firstinspires.ftc.teamcode.ServoCode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.security.cert.CRL;

public class ServoConDrive {

    private CRServo servoCon;
    public void ServoInit(HardwareMap hwMap) {
        servoCon = hwMap.get(CRServo.class, "servo_con");
    }

    public void setServoCon(double speed) {
        servoCon.setPower(speed);
    }
}
