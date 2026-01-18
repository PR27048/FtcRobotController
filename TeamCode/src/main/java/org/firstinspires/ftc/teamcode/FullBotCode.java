package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class FullBotCode extends OpMode {
    AWDcode drive = new AWDcode();
    double forward, strafe, rotate;
   // double Intake = 1.0;
    double Turret = -0.75;



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


      /*  if (gamepad1.left_bumper) {
            Turret = -0.75;
            drive.SetTurretPower(Turret);

        }

        if (gamepad1.right_bumper) {
            Turret = -0.9;
            drive.SetTurretPower(Turret);

        }*/


        drive.SetTurretPower();

        if (gamepad1.left_trigger > 0.1) {
            drive.SetIntakePower(1.0);
            drive.SetServoConFrontPower(-1.0);
            drive.setServoConPower(1.0);
            drive.SetServoConFrontPower(-1.0);
            drive.setIntakeServoPower(-1.0);

        }

        else if (gamepad1.right_trigger > 0.1) {
            drive.SetIntakePower(1.0);
            drive.SetServoConFrontPower(-1.0);
            drive.setServoConPower(-1.0);
            drive.SetServoConFrontPower(-1.0);
            drive.setIntakeServoPower(-1.0);

        }

        // NO TRIGGERS → everything OFF
        else {
            stop();
        }
        if (gamepad2.dpad_up) {
            drive.setHoodAngle(0);
        } else if(gamepad2.dpad_down) {
            drive.setHoodAngle(0.6);
        }
        double rightStick = gamepad2.right_stick_x;
        double clockwise = 0;
        double counterclockwise = 0;

        if (rightStick > 0.05) {
            clockwise -= rightStick;
        }
        if (rightStick < -0.05) {
            counterclockwise = rightStick;
        }

        drive.aimTurret(clockwise, counterclockwise);
    }

    @Override
    public void stop() {
        drive.SetIntakePower(0.0);
        //drive.SetTurretPower(0.0);
        drive.SetServoConFrontPower(0.0);
        drive.setServoConPower(0.0);
        drive.SetServoConFrontPower(0.0);
        drive.setIntakeServoPower(0.0);


    }

}