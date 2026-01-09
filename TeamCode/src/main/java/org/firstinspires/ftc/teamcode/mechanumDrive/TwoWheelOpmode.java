package org.firstinspires.ftc.teamcode.mechanumDrive;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

//@TeleOp
@Disabled
public class TwoWheelOpmode extends OpMode {
TwoWheelDrive driveObj = new TwoWheelDrive();

    @Override
    public void init() {
        driveObj.init2motors(hardwareMap);
    }

    @Override
    public void loop() {
        double forward = -gamepad1.left_stick_y;
        double rotate = gamepad1.right_stick_x;

        double BackLeftPower = forward + rotate;
        double BackRightPower = forward - rotate;

        driveObj.setMotorPower(BackLeftPower, BackRightPower);
    }
}
