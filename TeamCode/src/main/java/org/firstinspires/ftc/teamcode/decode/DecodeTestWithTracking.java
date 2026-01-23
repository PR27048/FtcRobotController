package org.firstinspires.ftc.teamcode.decode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import android.annotation.SuppressLint;
import android.util.Size;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
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
    double forward, strafe, rotate,speed;

    // double Intake = 1.0;
    double Turret = -0.75;

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
                .setCameraResolution(new Size(640, 480)) //width 640 height 480
                .addProcessor(aprilTagProcessor)
                .build();

        telemetry.addLine("AprilTag Tracking Initialized");

    }

    @Override
    public void loop() {

        forward = gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        rotate = gamepad1.right_stick_x;
        speed = 1.0;
        serviceHelper.drive(forward, strafe, rotate, speed);
        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();

        if (detections.isEmpty()) {
            turretServo.setPower(0);
            telemetry.addLine("No AprilTag detected");
            return;
        }

        AprilTagDetection tag = detections.get(0);
        if (gamepad2.a) {
            trackTag(tag);
        }
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

        // HOOKES LAWWWWWWW
        double power = KP * bearing;

        // get power
        power = Math.max(-MAX_POWER, Math.min(MAX_POWER, power));

        turretServo.setPower(power);
    }

    @SuppressLint("DefaultLocale")
    private void displayTelemetry(AprilTagDetection tag) {
        telemetry.addLine(String.format("ID: %d", tag.id));
        telemetry.addLine(String.format("Range: %.1f in", tag.ftcPose.range));
        telemetry.addLine(String.format("Bearing: %.1f deg", tag.ftcPose.bearing));
        telemetry.addLine(String.format("Yaw: %.1f deg", tag.ftcPose.yaw));

        //drive.SetIntakePower(Intake);



      /*  if (gamepad1.left_bumper) {
            Turret = -0.75;
            drive.SetTurretPower(Turret);

        }

        if (gamepad1.right_bumper) {
            Turret = -0.9;
            drive.SetTurretPower(Turret);

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
            serviceHelper.setHoodAngle(0);
        } else if(gamepad2.dpad_down) {
            serviceHelper.setHoodAngle(0.6);
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
        //drive.SetTurretPower(0.0);
        serviceHelper.SetServoConFrontPower(0.0);
        serviceHelper.setServoConPower(0.0);
        serviceHelper.setFeederPower(0.0);
        serviceHelper.SetServoConFrontPower(0.0);
        serviceHelper.setIntakeServoPower(0.0);
        turretServo.setPower(0);
        if (visionPortal != null) {
            visionPortal.close();
        }


    }
}
