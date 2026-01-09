package org.firstinspires.ftc.teamcode.mechanumDrive;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

//@TeleOp
@Disabled
public class MecanumFieldOrientedOpMode extends OpMode {
    MechanumFieldOrientedDrive drive = new MechanumFieldOrientedDrive();
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
