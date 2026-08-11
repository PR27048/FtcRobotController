package org.firstinspires.ftc.teamcode.outreach;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class TeleopOutreach extends OpMode {
    double forward,strafe,rotate,speed;

    UtilityOutreach utils = new UtilityOutreach();
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

        if (gamepad1.left_trigger > 0.1) {
            utils.Sethand(0);
        } else {utils.Sethand(0);}

        if (gamepad1.right_trigger > 0.1) {
            utils.Setarm(0.3);
        } else {utils.Setarm(0.5);}
    }

    @Override
    public void stop() {

    }

}
