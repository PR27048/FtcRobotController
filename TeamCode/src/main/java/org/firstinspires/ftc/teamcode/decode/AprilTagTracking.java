package org.firstinspires.ftc.teamcode.decode;

import android.annotation.SuppressLint;
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp
public class AprilTagTracking extends OpMode {

    private AprilTagProcessor aprilTagProcessor; //used to be private
    private VisionPortal visionPortal;
    private CRServo turretServo;

    //private static final double DEAD_ZONE_DEG = 2.0; //accepted range
    //private static final double SERVO_POWER = 0.09;

    private static final double DEAD_ZONE_DEG = 1.5;
    private static final double KP = 0.015;
    private static final double MAX_POWER = 0.25;

    @Override
    public void init() {
        turretServo = hardwareMap.get(CRServo.class, "servo_con_turret");

        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480)) //width 640 height 480
                .addProcessor(aprilTagProcessor)
                .build();

        telemetry.addLine("AprilTag Tracking Initialized");

    }

    @Override
    public void loop() {
        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();

        if (detections.isEmpty()) {
            turretServo.setPower(0);
            telemetry.addLine("No AprilTag detected");
            return;
        }

        AprilTagDetection tag = detections.get(0);

        trackTag(tag);
        displayTelemetry(tag);
    }

    private void trackTag(AprilTagDetection tag) {   //used to be private
        /*double bearing = tag.ftcPose.bearing;

        if (bearing > DEAD_ZONE_DEG) {
            //ADD: change servopower through equation based on bearing value
            turretServo.setPower(SERVO_POWER);
        }
        else if (bearing < -DEAD_ZONE_DEG) {
            //ADD: change servopower through equation based on bearing value
            turretServo.setPower(-SERVO_POWER);
        }
        else {
            turretServo.setPower(0);
        }*/

        double bearing = tag.ftcPose.bearing;

        // Dead zone
        if (Math.abs(bearing) < DEAD_ZONE_DEG) {
            turretServo.setPower(0);
            return;
        }

        // Proportional control
        double power = KP * bearing;

        // Clamp power
        power = Math.max(-MAX_POWER, Math.min(MAX_POWER, power));

        turretServo.setPower(power);
    }

    @SuppressLint("DefaultLocale")
    private void displayTelemetry(AprilTagDetection tag) {
        telemetry.addLine(String.format("ID: %d", tag.id));
        telemetry.addLine(String.format("Range: %.1f in", tag.ftcPose.range));
        telemetry.addLine(String.format("Bearing: %.1f deg", tag.ftcPose.bearing));
        telemetry.addLine(String.format("Yaw: %.1f deg", tag.ftcPose.yaw));
    }

    @Override
    public void stop() {
        turretServo.setPower(0);
        if (visionPortal != null) {
            visionPortal.close();
        }
    }
}