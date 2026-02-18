package org.firstinspires.ftc.teamcode.decode;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@TeleOp
public class DecodeTeleopWITHTRACKING extends OpMode {

    Limelightservicehelper serviceHelper = new Limelightservicehelper();

    double forward, strafe, rotate, speed;
    private boolean prevDpadUp = false;
    private boolean prevDpadDown = false;

    private boolean prevDpadLeft = false;
    private boolean prevDpadRight = false;

    private boolean initprevup = false;
    private boolean initprevdown = false;


    @Override
    public void init() {
        serviceHelper.init(hardwareMap, "FALSE");



    }
    public void init_loop() {
        boolean upPressed = gamepad1.dpad_up && !initprevup;
        boolean downPressed = gamepad1.dpad_down && !initprevdown;

        serviceHelper.updatePipelineMenu(upPressed, downPressed);

        initprevup = gamepad1.dpad_up;
        initprevdown = gamepad1.dpad_down;

        telemetry.addLine("\nSELECT BLUE/RED:");
        telemetry.addLine("\nDPAD_UP = RED");
        telemetry.addLine("\nDPAD_DOWN = BLUE");
        telemetry.addData("CURRENT SELECTION:", serviceHelper.getCurrentPipeline());


        telemetry.update();

    }

    @Override
    public void loop() {

        // --- Drive ---
        forward = gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        rotate = gamepad1.right_stick_x;
        speed = 1.0;
        serviceHelper.drive(forward, strafe, rotate, speed);

        // --- Turret Tracking ---
        serviceHelper.trackWithLimelight();
       // telemetry.addData("Turret Angle: ", serviceHelper.turretAngle);


        // --- Adjust KP in TeleOp ---
        if (gamepad1.dpad_up && !prevDpadUp) {
            Limelightservicehelper.KP += 0.0001;
        }
        if (gamepad1.dpad_down && !prevDpadDown) {
            Limelightservicehelper.KP -= 0.0001;
        }

        if (gamepad1.dpad_right && !prevDpadRight) {
            Limelightservicehelper.KD += 0.00001;
        }
        if (gamepad1.dpad_left && !prevDpadLeft) {
            Limelightservicehelper.KD -= 0.00001;
        }
        prevDpadUp = gamepad1.dpad_up;
        prevDpadDown = gamepad1.dpad_down;
        prevDpadLeft = gamepad1.dpad_left;
        prevDpadRight = gamepad1.dpad_right;

        // --- Telemetry ---
        telemetry.addData("KP Tuning Constant: ", Limelightservicehelper.KP);
        telemetry.addData("KD Tuning Constant: ", Limelightservicehelper.KD);

        telemetry.addData("Turret Servo Position: ", serviceHelper.getTurretPosition());

        telemetry.update();

        // --- Intake / Feeder Control ---
        if (gamepad1.left_trigger > 0.1) {
            serviceHelper.SetIntakePower(1.0);
            serviceHelper.SetServoConFrontPower(-1.0);
            serviceHelper.setFeederPower(0.7);
            serviceHelper.setIntakeServoPower(-1.0);
            //serviceHelper.SetTurretVelocity();

        } else if (gamepad1.right_trigger > 0.1) {
            serviceHelper.SetTurretVelocity();

            if (serviceHelper.isTurretAtSpeed()) {
                serviceHelper.SetIntakePower(1.0);
                serviceHelper.SetServoConFrontPower(-1.0);
                serviceHelper.setFeederPower(-0.7);
                serviceHelper.setIntakeServoPower(-1.0);
            }

        } else {
            serviceHelper.SetIntakePower(0.0);
            serviceHelper.SetServoConFrontPower(0.0);
            serviceHelper.setFeederPower(0.0);
            serviceHelper.setIntakeServoPower(0.0);
            serviceHelper.SetTurretOFF();
        }

        /*if (gamepad2.a) {
            serviceHelper.ReverseTurret();
        }
*/
        // --- Hood Control ---
        if (gamepad2.dpad_up) serviceHelper.setHoodAngle(0.45);
        if (gamepad2.dpad_down) serviceHelper.setHoodAngle(-0.15);

        // --- Manual turret aim ---
        double rightStick = gamepad2.right_stick_x;
        double clockwise = 0, counterclockwise = 0;
        if (rightStick > 0.05) clockwise -= rightStick;
        if (rightStick < -0.05) counterclockwise = rightStick;

       // serviceHelper.aimTurret(clockwise, counterclockwise);
    }

    @Override
    public void stop() {
        serviceHelper.SetIntakePower(0.0);
        serviceHelper.SetServoConFrontPower(0.0);
        serviceHelper.setServoConPower(0.0);
        serviceHelper.setFeederPower(0.0);
        serviceHelper.setIntakeServoPower(0.0);
        serviceHelper.stopLimelight();
    }
}
