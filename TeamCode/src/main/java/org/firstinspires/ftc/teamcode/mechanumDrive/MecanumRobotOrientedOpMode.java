package org.firstinspires.ftc.teamcode.mechanumDrive;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

//@TeleOp
@Disabled
public class MecanumRobotOrientedOpMode extends OpMode {
    MechanumRobotOrientedDrive drive = new MechanumRobotOrientedDrive();
    double forward, strafe, rotate;


    @Override
    public void init() {
        drive.init(hardwareMap);
    }

    @Override
    public void loop() {
        forward = gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        rotate = gamepad1.right_stick_x;

        drive.drive(forward, strafe, rotate);
    }

}
