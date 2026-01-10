package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class FullBotCode extends OpMode {
    AWDcode drive = new AWDcode();
    double forward, strafe, rotate;
   // double Intake = 0.6;
   double Turret = -0.8;


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
        //drive.SetIntakePower(Intake);

        if (gamepad1.left_bumper) {
            Turret = -0.9;
        }

        if (gamepad1.right_bumper) {
            Turret = -0.8;
        }

        drive.SetTurretPower(Turret);


        if (gamepad1.left_trigger > 0.1) {
            drive.SetIntakePower(0.8);
            drive.SetServoConFrontPower(-1.0);
            drive.setServoConPower(1.0);
            drive.SetServoConFrontPower(-1.0);
            drive.setIntakeServoPower(-1.0);
        }

        // RIGHT TRIGGER = intake ON, back servo reversed
        else if (gamepad1.right_trigger > 0.1) {
            drive.SetIntakePower(0.8);
            drive.SetServoConFrontPower(-1.0);
            drive.setServoConPower(-1.0);
            drive.SetServoConFrontPower(-1.0);
            drive.setIntakeServoPower(-1.0);

        }

        // NO TRIGGERS → everything OFF
        else {
            stop();
        }


    }
    @Override
    public void stop() {
        drive.SetIntakePower(0.0);
        drive.SetTurretPower(0.0);
        drive.SetServoConFrontPower(0.0);
        drive.setServoConPower(0.0);
        drive.SetServoConFrontPower(0.0);
        drive.setIntakeServoPower(0.0);


    }

}