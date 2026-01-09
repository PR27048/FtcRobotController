package org.firstinspires.ftc.teamcode.mechanumDrive;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.concurrent.ArrayBlockingQueue;

public class TwoWheelDrive {
    private DcMotor BackLeft;
    private DcMotor BackRight;

    public void init2motors(HardwareMap hwMap) {
        BackLeft = hwMap.get(DcMotor.class, "back_left");
        BackRight = hwMap.get(DcMotor.class, "back_right");

        BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        BackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    public void setMotorPower(double BackLeftPower,double BackRightPower) {
        double largest = 1.00;
        largest = Math.max(largest, Math.abs(BackLeftPower));
        largest = Math.max(largest, Math.abs(BackRightPower));

        BackLeft.setPower(BackLeftPower / largest);
        BackRight.setPower(BackRightPower / largest);

    }
}
