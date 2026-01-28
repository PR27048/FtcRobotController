package org.firstinspires.ftc.teamcode.decode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp
public class DecodeTeleOPAaditya extends OpMode {

    ServiceHelperAaditya serviceHelper = new ServiceHelperAaditya();
    double forward, strafe, rotate,speed;

    // double Intake = 1.0;
    double Turret = -0.75;

    @Override
    public void init() {
        serviceHelper.init(hardwareMap, "FALSE");

    }

    @Override
    public void loop() {

        forward = gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        rotate = gamepad1.right_stick_x;
        speed = 1.0;
        serviceHelper.drive(forward, strafe, rotate, speed);


        //drive.SetIntakePower(Intake);



      /*  if (gamepad1.left_bumper) {
            Turret = -0.75;
            drive.SetTurretPower(Turret);

        }

        if (gamepad1.right_bumper) {
            Turret = -0.9;
            drive.SetTurretP    ower(Turret);

        }*/


        serviceHelper.SetTurretPower();

        if (gamepad1.left_trigger > 0.1) {
            serviceHelper.SetIntakePower(1.0);
            serviceHelper.SetServoConFrontPower(-1.0);
            serviceHelper.setFeederPower(0.7);
            serviceHelper.SetServoConFrontPower(-1.0);
            serviceHelper.setIntakeServoPower(-1.0);
            serviceHelper.SetTurretPower();

        }

        else if (gamepad1.right_trigger > 0.1) {
            serviceHelper.SetIntakePower(1.0);
            serviceHelper.SetServoConFrontPower(-1.0);
            serviceHelper.setFeederPower(-0.7);
            serviceHelper.SetServoConFrontPower(-1.0);
            serviceHelper.setIntakeServoPower(-1.0);
            serviceHelper.SetTurretPowerAccel();


        }

        // NO TRIGGERS → everything OFF
        else {
            stop();
        }
        if (gamepad2.dpad_up) {
            serviceHelper.setHoodAngle(0.15);
        }
        if (gamepad2.dpad_down) {
            serviceHelper.setHoodAngle(-0.15);
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

        serviceHelper.aimTurret(clockwise, counterclockwise);

    }

    @Override
    public void stop() {
        serviceHelper.SetIntakePower(0.0);
        serviceHelper.SetServoConFrontPower(0.0);
        serviceHelper.setServoConPower(0.0);
        serviceHelper.setFeederPower(0.0);
        serviceHelper.SetServoConFrontPower(0.0);
        serviceHelper.setIntakeServoPower(0.0);


    }
}