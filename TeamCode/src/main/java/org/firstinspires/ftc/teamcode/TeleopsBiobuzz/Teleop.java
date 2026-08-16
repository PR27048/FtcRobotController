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
        forward = Math.pow(gamepad1.left_stick_y, 3);
        strafe = Math.pow(gamepad1.left_stick_x,3);
        rotate = Math.pow(gamepad1.right_stick_x,3);

        speed = 1.0; //full

        utils.drive(forward,strafe,rotate,speed);

        if(gamepad1.left_trigger >0.1) {
            utils.Setintake(1);
        } else{utils.Setintake(0);}
        if(gamepad1.right_trigger >0.1) {
            utils.Setintake(-1);
        } else{utils.Setintake(0);}
    }

    @Override
    public void stop() {
        utils.Setintake(0);
    }

}
