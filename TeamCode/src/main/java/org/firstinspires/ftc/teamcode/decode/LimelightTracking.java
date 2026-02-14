package org.firstinspires.ftc.teamcode.decode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;

@TeleOp
public class LimelightTracking extends OpMode {

    private Limelight3A limelight;
    private CRServo turretServo;

    private static final double DEAD_ZONE_DEG = 1.5;
    private static double KP = 0.02; // tuning constant
    private static final double MAX_POWER = 0.25;

    @Override
    public void init() {

        turretServo = hardwareMap.get(CRServo.class, "servo_con_turret");

        // Get Limelight from hardware map
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        limelight.pipelineSwitch(0); // Have to configure this still
        limelight.start();

        telemetry.addLine("Limelight FTC Tracking Initialized");
    }

    @Override
    public void loop() {

        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) {
            turretServo.setPower(0);
            telemetry.addLine("No Target Detected");
            return;
        }

        double tx = result.getTx();  // horizontal offset in degrees
        //tuning
        if (gamepad1.dpad_down) {
            KP-=0.001;
        }
        if (gamepad1.dpad_up) {
            KP+=0.001;
        }
        trackTarget(tx);
        telemetry.addLine("POWER CONSTANT: " + KP);
        telemetry.addData("tx", tx);

    }

    private void trackTarget(double tx) {

        if (Math.abs(tx) < DEAD_ZONE_DEG) {
            turretServo.setPower(0);
            return;
        }

        double power = KP * tx;
        power = Math.max(-MAX_POWER, Math.min(MAX_POWER, power));

        turretServo.setPower(-power);
    }

    @Override
    public void stop() {
        turretServo.setPower(0);
        limelight.stop();
    }
}
