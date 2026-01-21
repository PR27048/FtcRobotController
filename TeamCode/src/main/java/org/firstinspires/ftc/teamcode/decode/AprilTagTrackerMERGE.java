package org.firstinspires.ftc.teamcode.decode;

import android.util.Size;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class AprilTagTrackerMERGE {

    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;
    private CRServo turretServo;

    private static final double DEAD_ZONE_DEG = 2.0;
    private static final double SERVO_POWER = 0.09;

    public void init(HardwareMap hardwareMap) {
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
                .setCameraResolution(new Size(640, 480))
                .addProcessor(aprilTagProcessor)
                .build();
    }

    public void update() {
        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
        if (detections.isEmpty()) {
            turretServo.setPower(0);
            return;
        }

        AprilTagDetection tag = detections.get(0);
        trackTag(tag);
    }

    private void trackTag(AprilTagDetection tag) {
        double bearing = tag.ftcPose.bearing;

        if (bearing > DEAD_ZONE_DEG) {
            turretServo.setPower(SERVO_POWER);
        } else if (bearing < -DEAD_ZONE_DEG) {
            turretServo.setPower(-SERVO_POWER);
        } else {
            turretServo.setPower(0);
        }
    }

    public AprilTagDetection getBestTag() {
        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
        return detections.isEmpty() ? null : detections.get(0);
    }

    public void stop() {
        turretServo.setPower(0);
        if (visionPortal != null) {
            visionPortal.close();
        }
    }
}
