package org.firstinspires.ftc.teamcode.mecanumwheels;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp
public class TwoMotorDriveMain extends OpMode {
    TwoMotorDrive driveObj = new TwoMotorDrive();
    public HardwareMap hwMap;

    @Override
    public void init() {
        driveObj.init(hwMap);
    }

    @Override
    public void loop() {
        double leftPower, rightPower;
        double forward, right;

        forward = -gamepad1.left_stick_y;
        right = gamepad1.left_stick_x;

        leftPower = forward + right;
        rightPower = forward - right;

        driveObj.setMotorsPower(leftPower,rightPower);
    }
}
