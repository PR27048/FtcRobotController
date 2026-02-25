package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;

@TeleOp
@Disabled
public class LimelightTracking extends OpMode {

    private Limelight3A limelight;
    private Servo turretServo;
    private ElapsedTime driveTimer = new ElapsedTime();

    private double KP = 0.03;        // Proportional gain
    private double KD = 0.002;       // Derivative gain (start small)
    private final double DEAD_ZONE_DEG = 1.5;
    private final double MAX_POWER = 0.25;

    private double lastTx = 0;
    private double lastTime = 0;

    private boolean prevDpadUp = false;
    private boolean prevDpadDown = false;
   // double currentPosition = turretServo.getPosition();

    @Override
    public void init() {

        turretServo = hardwareMap.get(Servo.class, "servo_con_turret");

        // Get Limelight from hardware map
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        limelight.pipelineSwitch(0); // Have to configure this still
        limelight.start();

        telemetry.addLine("Limelight FTC Tracking Initialized");
        driveTimer.reset();
        lastTime = driveTimer.seconds();
    }

    @Override
    public void loop() {

        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) {
            telemetry.addLine("No Target Detected");
            return;
        }


        double tx = result.getTx();  // horizontal offset in degrees

        //TUNING P VALUE
        if (gamepad1.dpad_up && !prevDpadUp) {
            KP += 0.001;
        }
        if (gamepad1.dpad_down && !prevDpadDown) {
            KP -= 0.001;
        }

        prevDpadUp = gamepad1.dpad_up;
        prevDpadDown = gamepad1.dpad_down;

        trackTarget(tx);
        telemetry.addLine("POWER CONSTANT: " + KP);
        telemetry.addLine("TurretServo position: " + turretServo);
        telemetry.addData("tx", tx);

    }

    private void trackTarget(double tx) {
        //APRIL TAG TRACKING LOGIC


        if (Math.abs(tx) < DEAD_ZONE_DEG) {
            lastTx = tx;
            return; // positional servo holds its last position automatically
        }

// Proportional adjustment
        double adjustment = KP * tx;
        double min = 0.47;
        double max = 0.51;

// Reverse direction if needed (same as -power before)
        adjustment = -adjustment;

// Get current servo position
        double currentPosition = turretServo.getPosition();

// Calculate new position
        double newPosition = currentPosition + adjustment;

// Clamp to valid servo range
        newPosition = Math.max(min, Math.min(max, newPosition));

// Move servo
        turretServo.setPosition(newPosition);

        lastTx = tx;

    }

    @Override
    public void stop() {
        limelight.stop();
    }
}
