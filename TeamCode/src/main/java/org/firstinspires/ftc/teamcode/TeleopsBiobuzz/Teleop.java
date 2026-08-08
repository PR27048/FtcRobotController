package org.firstinspires.ftc.teamcode.TeleopsBiobuzz;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class Teleop extends OpMode {
    double forward,strafe,rotate,speed;

    Utilities utils = new Utilities();
    @Override
    public void init() {
        utils.init(hardwareMap, "FALSE");

        telemetry.addLine("Press start");
        telemetry.update();
    }


    @Override
    public void loop() {
        forward = gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        rotate = gamepad1.right_stick_x;
        speed = 1.0; //full

        utils.drive(forward,strafe,rotate,speed);
    }

    @Override
    public void stop() {

    }

}
