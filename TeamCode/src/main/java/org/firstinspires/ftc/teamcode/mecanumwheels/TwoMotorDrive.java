package org.firstinspires.ftc.teamcode.mecanumwheels;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class TwoMotorDrive {
    private DcMotor leftMotor, rightMotor;

    public void init(HardwareMap hwMap) {
        leftMotor = hwMap.get(DcMotor.class, "left_motor");
        rightMotor = hwMap.get(DcMotor.class, "right_motor");

        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setMotorsPower(double leftPower, double rightPower) {
        double largest = 1.0;

        largest = Math.max(largest, Math.abs(leftPower));
        largest = Math.max(largest, Math.abs(rightPower));

        leftMotor.setPower(leftPower / largest);
        rightMotor.setPower(rightPower / largest);
    }
}
