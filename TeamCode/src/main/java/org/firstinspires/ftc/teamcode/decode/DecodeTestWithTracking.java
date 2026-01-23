package org.firstinspires.ftc.teamcode.decode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

import android.annotation.SuppressLint;
import android.util.Size;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp
public class DecodeTestWithTracking extends OpMode {

    ServiceHelperAaditya serviceHelper = new ServiceHelperAaditya();

    double forward, strafe, rotate, speed;

    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;
    private CRServo turretServo;

    private static final double DEAD_ZONE_DEG = 1.5;
    private static final double KP = 0.015; //HAVE TO TUNE THIS VALUE
    private static final double MAX_POWER = 0.25;

    @Override
    public void init() {

        turretServo = hardwareMap.get(CRServo.class, "servo_con_turret");
        serviceHelper.init(hardwareMap, "FALSE");

        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .addProcessor(aprilTagProcessor)
                .build();

        telemetry.addLine("AprilTag Tracking Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {


        // DRIVE

        forward = gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        rotate = gamepad1.right_stick_x;
        speed = 1.0;
        serviceHelper.drive(forward, strafe, rotate, speed);
        serviceHelper.SetTurretPower();

        // DRIVER MECHANISMS

        handleIntake();
        handleHood();

        // APRILTAG DETECTION

        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();

        boolean autoTurret = gamepad2.a && !detections.isEmpty();

        if (autoTurret) {
            AprilTagDetection tag = detections.get(0);
            trackTag(tag);
            displayTelemetry(tag);
        } else {
            handleManualTurret();

            if (detections.isEmpty()) {
                telemetry.addLine("No AprilTag detected");
            } else {
                displayTelemetry(detections.get(0));
            }
        }

        telemetry.update();
    }

    // AUTO TURRET (APRILTAG)
    private void trackTag(AprilTagDetection tag) {

        double bearing = tag.ftcPose.bearing;

        if (Math.abs(bearing) < DEAD_ZONE_DEG) {
            turretServo.setPower(0);
            return;
        }

        double power = KP * bearing;
        power = Math.max(-MAX_POWER, Math.min(MAX_POWER, power));

        turretServo.setPower(power);
    }


    // MANUAL TURRET

    private void handleManualTurret() {

        double stick = gamepad2.right_stick_x;

        if (Math.abs(stick) > 0.05) {
            turretServo.setPower(stick * -0.6);
        } else {
            turretServo.setPower(0);
        }
    }

    // INTAKE / FEEDER
    private void handleIntake() {

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
        else {
            stopIntakeAndFeeders();
        }
    }

    private void stopIntakeAndFeeders() {
        serviceHelper.SetIntakePower(0.0);
        serviceHelper.SetServoConFrontPower(0.0);
        serviceHelper.setServoConPower(0.0);
        serviceHelper.setFeederPower(0.0);
        serviceHelper.setIntakeServoPower(0.0);
    }


    // HOOD

    private void handleHood() {
        if (gamepad2.dpad_up) {
            serviceHelper.setHoodAngle(0);
        } else if (gamepad2.dpad_down) {
            serviceHelper.setHoodAngle(0.5);
        }
    }


    // TELEMETRY

    @SuppressLint("DefaultLocale")
    private void displayTelemetry(AprilTagDetection tag) {
        telemetry.addLine(String.format("ID: %d", tag.id));
        telemetry.addLine(String.format("Range: %.1f in", tag.ftcPose.range));
        telemetry.addLine(String.format("Bearing: %.1f deg", tag.ftcPose.bearing));
        telemetry.addLine(String.format("Yaw: %.1f deg", tag.ftcPose.yaw));
    }

    @Override
    public void stop() {

        stopIntakeAndFeeders();
        turretServo.setPower(0);

        if (visionPortal != null) {
            visionPortal.close();
        }
    }
}
