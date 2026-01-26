package org.firstinspires.ftc.teamcode.decode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Disabled
@TeleOp
public class DecodeTeleOPNishanth extends OpMode {

    ServiceHelperNishanth serviceHelper = new ServiceHelperNishanth();

    double forward, strafe, rotate;
    double IntakePower = 1.0;
    double TurretPower = 0.47;

    double rightStick;
    double clockwise = 0;
    double counterclockwise = 0;

    @Override
    public void init() {

        serviceHelper.init(hardwareMap);
    }

    @Override
    public void start() {

        serviceHelper.SetIntakePower(IntakePower);
    }

    @Override
    public void loop() {

        forward = gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        rotate = gamepad1.right_stick_x;

        serviceHelper.drive(forward, strafe, rotate);

        serviceHelper.SetServoConIntakePower(-1.0);
        serviceHelper.SetServoConFrontPower(-0.8);

        if (gamepad2.left_bumper) {
            TurretPower = 0.625;
            telemetry.addData("long range", TurretPower);
        }

        if (gamepad2.right_bumper) {
            TurretPower = 0.75;
            telemetry.addData("mid range", TurretPower);
        }

        serviceHelper.SetTurretPower(TurretPower);

         rightStick = gamepad2.right_stick_x;
         clockwise = 0;
         counterclockwise = 0;

        if (rightStick > 0.05) {
            clockwise = rightStick;
        }
        if (rightStick < -0.05) {
            counterclockwise = -rightStick;
        }

        serviceHelper.aimTurret(clockwise, counterclockwise);

        double leftStick = gamepad2.left_stick_y;
        double up = 0;
        double down = 0;

        if (leftStick > 0.05) {
            up = leftStick;
        }
        if (leftStick < -0.05) {
            down = -leftStick;
        }

        serviceHelper.hoodServo(up,down);

        if (gamepad1.right_trigger > 0.05) {
            serviceHelper.SetBackFeederPower(-0.8);
        }
        else {
            serviceHelper.SetBackFeederPower(0.8);
        }

        if (gamepad2.left_trigger > 0.05) {
            serviceHelper.SetIntakePower(0.0);
            serviceHelper.SetServoConIntakePower(0.0);
        }

        else if (gamepad2.right_trigger > 0.05) {
            serviceHelper.SetIntakePower(0.8);
            serviceHelper.SetServoConIntakePower(-0.8);
        }
    }

    @Override
    public void stop() {

        serviceHelper.SetIntakePower(0.0);
        serviceHelper.SetTurretPower(0.0);
        serviceHelper.SetServoConFrontPower(0.0);
        serviceHelper.SetBackFeederPower(0.0);
        serviceHelper.SetServoConIntakePower(0.0);

    }


}
