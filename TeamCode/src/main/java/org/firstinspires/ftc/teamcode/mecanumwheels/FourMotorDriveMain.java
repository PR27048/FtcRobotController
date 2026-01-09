package org.firstinspires.ftc.teamcode.mecanumwheels;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp
public class FourMotorDriveMain extends OpMode {
    RobotOrientationDrive driveObj = new RobotOrientationDrive();
    double forward, strafe, rotate;
    public HardwareMap hwMap;


    @Override
    public void init() {

        driveObj.roInitializeMotors(hwMap);
    }

    @Override
    public void loop () {
        forward = gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        rotate = gamepad1.right_stick_x;

        driveObj.roDrive(forward, strafe, rotate);

    }
}
