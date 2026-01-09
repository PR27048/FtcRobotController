package org.firstinspires.ftc.teamcode.testcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class DcMotorFuc {

    public DcMotor motor;

    public void setSpeed(double speed) {
        motor.setPower(speed);
    }

}
