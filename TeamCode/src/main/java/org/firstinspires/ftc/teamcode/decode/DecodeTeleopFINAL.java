package org.firstinspires.ftc.teamcode.decode;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.Range;

@TeleOp
public class DecodeTeleopFINAL extends OpMode {

    FullFieldHelper serviceHelper = new FullFieldHelper();

    double forward, strafe, rotate, speed;
    private boolean prevDpadUp = false;
    private boolean prevDpadDown = false;

    private boolean prevDpadLeft = false;
    private boolean prevDpadRight = false;

    private boolean initprevup = false;
    private boolean initprevdown = false;
    private boolean hasRumbled = false;


    @Override
    public void init() {
        serviceHelper.init(hardwareMap, "FALSE");



    }
    public void init_loop() {
        boolean upPressed = gamepad1.dpad_up && !initprevup;
        boolean downPressed = gamepad1.dpad_down && !initprevdown;


        initprevup = gamepad1.dpad_up;
        initprevdown = gamepad1.dpad_down;
        String selection = "";
        telemetry.addLine("\nSELECT BLUE/RED:");
        telemetry.addLine("\nDPAD_UP = RED");
        telemetry.addLine("\nDPAD_DOWN = BLUE");
        if (serviceHelper.getCurrentPipeline() == 0) {
            selection = "RED";
        } else {
            selection = "BLUE";
        }

        telemetry.addData("CURRENT SELECTION:", selection);

        telemetry.update();
        serviceHelper.updatePipelineMenu(upPressed, downPressed);

    }

    @Override
    public void loop() {
        double x = serviceHelper.getDistance(); // distance from cam to tag
        double HorizontalDistance = x * Math.cos(Math.toRadians(23.5)); // adjust if needed
        double tx = serviceHelper.getTX(); // horizontal angle to tag, in degrees
        double heading = serviceHelper.getHeading(); // robot heading in radians

        // --- Compute Robot Field Position ---
        double txRad = Math.toRadians(tx);
        // Robot-relative coordinates to tag
        double xRel = HorizontalDistance * Math.cos(txRad);
        double yRel = HorizontalDistance * Math.sin(txRad);

        // Rotate by robot heading to get field offset
        double cosH = Math.cos(heading);
        double sinH = Math.sin(heading);
        double xOffset = xRel * cosH - yRel * sinH;
        double yOffset = xRel * sinH + yRel * cosH;

        // Known tag field position (example: put your actual tag coordinates here)
        double x_tag = 72.0; // inches or your unit
        double y_tag = 36.0;

        // Compute robot field coordinates
        double robotX = x_tag - xOffset;
        double robotY = y_tag - yOffset;

        // --- Drive Controls ---
        forward = gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        rotate = gamepad1.right_stick_x;
        speed = 1.0;
        serviceHelper.drive(forward, strafe, rotate, speed);
        serviceHelper.setlimelightpipeline();
        serviceHelper.currentPipeline = serviceHelper.getCurrentPipeline();
        // --- Turret Tracking ---
        serviceHelper.trackWithLimelight();

        prevDpadUp = gamepad1.dpad_up;
        prevDpadDown = gamepad1.dpad_down;

        // --- Shooter Calculations ---
        double Velocity = 0;
        double Hoodpos = 0;

        if (!Double.isNaN(x) && x > 0) {
            Velocity = Range.clip(
                    (-0.00000622468) * x * x * x * x
                            + 0.00220552 * x * x * x
                            - 0.270902 * x * x
                            + 17.19245 * x
                            + 593.70277,
                    0, 1440
            );

            Hoodpos = Range.clip(
                    (2.02902e-8) * x * x * x * x
                            - 0.0000069818 * x * x * x
                            + 0.000863395 * x * x
                            - 0.0479336 * x
                            + 1.75901,
                    0.4, 1.0
            );

            serviceHelper.setHood(Hoodpos);
            serviceHelper.SetTurretVelocity(Velocity);

        }
        boolean tagLost = serviceHelper.LostTag();


        if (tagLost) {
            gamepad2.rumble(500);
            hasRumbled = true;
        }
        if (!tagLost) {
            gamepad2.stopRumble();
            hasRumbled = false;

        }
        // --- Telemetry ---
        telemetry.addData("Distance to Tag", "%.2f", x);
        telemetry.addData("Horizontal Distance", "%.2f", HorizontalDistance);
        telemetry.addData("Robot X", "%.2f", robotX);
        telemetry.addData("Robot Y", "%.2f", robotY);
        telemetry.addData("Heading (rad)", "%.2f", heading);
        telemetry.addData("LostTag?", serviceHelper.LostTag());

        telemetry.update();

        // --- Intake / Feeder ---
        if (gamepad1.left_trigger > 0.1) {
            serviceHelper.SetIntakePower(1.0);
            serviceHelper.SetServoConFrontPower(-0.6);
            serviceHelper.setFeederPower(1.0);
            serviceHelper.setIntakeServoPower(-1.0);
        } else if (gamepad1.right_trigger > 0.1) {
            serviceHelper.SetTurretVelocity(Velocity);
            if (!Double.isNaN(x) && x > 15 && serviceHelper.isTurretAtSpeed(Velocity)) {
                    serviceHelper.SetIntakePower(1.0);
                    serviceHelper.SetServoConFrontPower(-1.0);
                    serviceHelper.setFeederPower(-1.0);
                    serviceHelper.setIntakeServoPower(-1.0);

            }
        } else {
            serviceHelper.SetIntakePower(0.0);
            serviceHelper.SetServoConFrontPower(0.0);
            serviceHelper.setFeederPower(0.0);
            serviceHelper.setIntakeServoPower(0.0);
        }

        // --- Manual Turret Aim ---
        double rightStick = gamepad2.right_stick_x;
        double clockwise = 0, counterclockwise = 0;
        if (rightStick > 0.05) clockwise -= rightStick;
        if (rightStick < -0.05) counterclockwise = rightStick;
        serviceHelper.aimTurret(clockwise, counterclockwise);
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
