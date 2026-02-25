package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.goBildaPinpoint.GoBildaPinpointDriver;

import java.util.Locale;

@TeleOp
public class OdometryTracker extends OpMode {

    GoBildaPinpointDriver odo;
    private Servo turretServo;

    // Red goal coordinates in inches
    double redGoalXInches = 144;
    double redGoalYInches = 144;

    // Convert inches to mm for GoBilda Pinpoint
    double redGoalXmm = redGoalXInches * 25.4;
    double redGoalYmm = redGoalYInches * 25.4;

    @Override
    public void init() {
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        turretServo = hardwareMap.get(Servo.class, "turretServo");

        // Setup odometry
        odo.setOffsets(-120.65, -196.85, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Update odometry
        odo.update();

        Pose2D pos = odo.getPosition();

        double robotX = pos.getX(DistanceUnit.MM);
        double robotY = pos.getY(DistanceUnit.MM);
        double robotHeading = pos.getHeading(AngleUnit.RADIANS); // radians

        // Calculate angle from robot to red goal
        double angleToGoal = Math.atan2(redGoalYmm - robotY, redGoalXmm - robotX); // radians

        // Calculate relative angle for servo (0 = forward)
        double relativeAngle = angleToGoal - robotHeading;

        // Normalize relativeAngle to -PI..PI
        while (relativeAngle > Math.PI) relativeAngle -= 2 * Math.PI;
        while (relativeAngle < -Math.PI) relativeAngle += 2 * Math.PI;

        // Map relative angle to servo position [-90° -> 0, 0° -> 0.5, +90° -> 1.0]
        double servoPos = 0.5 + (relativeAngle / Math.PI) * 0.5;

        // Clamp to 0-1
        servoPos = Math.max(0.0, Math.min(1.0, servoPos));

        // Set servo
        turretServo.setPosition(servoPos);

        // Telemetry
        telemetry.addData("Robot X (mm)", robotX);
        telemetry.addData("Robot Y (mm)", robotY);
        telemetry.addData("Heading (deg)", Math.toDegrees(robotHeading));
        telemetry.addData("Servo Pos", servoPos);
        telemetry.update();

        // Optional gamepad controls
        if (gamepad1.a) {
            odo.resetPosAndIMU();
        }
        if (gamepad1.b) {
            odo.recalibrateIMU();
        }
    }
}