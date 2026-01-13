package org.firstinspires.ftc.teamcode;

import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.stateBot2526Drive;

@TeleOp
public class stateBot2526OpMode extends OpMode {
    stateBot2526Drive drive = new stateBot2526Drive();
    double forward, strafe, rotate;
    double IntakePower = 0.8;
    double TurretPower = 0.75;


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
        drive.SetIntakePower(IntakePower);
        drive.SetServoConIntakePower(-0.8);
        drive.SetServoConFrontPower(-0.8);

        if (gamepad2.left_bumper) {
            TurretPower = 0.85;
            telemetry.addData("long range", TurretPower);
        }

        if (gamepad2.right_bumper) {
            TurretPower = 0.75;
            telemetry.addData("mid range", TurretPower);
        }

        drive.SetTurretPower(TurretPower);

        double rightStick = gamepad2.right_stick_x;
        double clockwise = 0;
        double counterclockwise = 0;

        if (rightStick > 0.05) {
            clockwise = rightStick;
        }
        if (rightStick < -0.05) {
            counterclockwise = -rightStick;
        }

        drive.aimTurret(clockwise, counterclockwise);

        if (gamepad2.right_trigger > 0.1) {
            drive.SetServoConBackPower(-0.8);
        }
        else {
            drive.SetServoConBackPower(0.8);
        }

        if (gamepad1.left_trigger > 0.1) {
            drive.SetIntakePower(0.0);
            drive.SetServoConIntakePower(0.0);
        }

        if (gamepad1.right_trigger > 0.1) {
            drive.SetIntakePower(0.8);
            drive.SetServoConIntakePower(-0.8);
        }

    }
    @Override
    public void stop() {
        drive.SetIntakePower(0.0);
        drive.SetTurretPower(0.0);
        drive.SetServoConFrontPower(0.0);
        drive.SetServoConBackPower(0.0);
        drive.SetServoConIntakePower(0.0);
    }

}
