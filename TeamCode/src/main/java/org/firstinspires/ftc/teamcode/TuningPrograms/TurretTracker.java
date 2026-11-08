package org.firstinspires.ftc.teamcode.TuningPrograms;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Teleops.Utilities;

// Change this import only if your GoBildaPinpointDriver is in a different package.
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

@TeleOp
@Disabled
public class TurretTracker extends OpMode {

    private static final String PINPOINT_NAME = "pinpoint";

    private static final double NORMAL_GATE_POSITION = 0.07;
    private static final double ALIGN_GATE_POSITION = 0.70;
    private static final double ALIGN_KP = 0.026;
    private static final double MAX_ALIGN_TURN = 0.40;
    private static final double TRIGGER_DEADBAND = 0.10;

    private GoBildaPinpointDriver odo;
    private final Utilities utils = new Utilities();

    private boolean previousDpadUp;
    private boolean previousDpadDown;

    private double robotX;
    private double robotY;
    private double robotHeadingDeg;

    @Override
    public void init() {
        utils.init(hardwareMap, "FALSE");
        utils.SetGate(NORMAL_GATE_POSITION);

        // Initialize turret servo home position to center (0.5)
        utils.SetTurretPos(0.5);

        odo = hardwareMap.get(GoBildaPinpointDriver.class, PINPOINT_NAME);

        // setOffsets() expects millimeters in the standard goBILDA driver.
        odo.setOffsets(
                6.502978707861712,
                -2.696905361385796,
                DistanceUnit.INCH
        );

        odo.setEncoderResolution(
                GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD
        );

        odo.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );

        // Reset once during INIT. Do not reset in loop().
        odo.resetPosAndIMU();

        telemetry.addLine("Pinpoint & Turret initialized");
        telemetry.addLine("Place the robot at the desired field origin.");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        boolean upPressed = gamepad1.dpad_up && !previousDpadUp;
        boolean downPressed = gamepad1.dpad_down && !previousDpadDown;

        previousDpadUp = gamepad1.dpad_up;
        previousDpadDown = gamepad1.dpad_down;

        utils.updatePipelineMenu(upPressed, downPressed);

        String selection = utils.getCurrentPipeline() == 2 ? "RED" : "BLUE";
        telemetry.addLine("Select Alliance Color:");
        telemetry.addLine("DPAD_UP = RED");
        telemetry.addLine("DPAD_DOWN = BLUE");
        telemetry.addData("Current selection", selection);
        telemetry.update();
    }

    @Override
    public void start() {
        // Refresh once at start without resetting the established origin.
        odo.update();
    }

    @Override
    public void loop() {
        // Call exactly once near the top of every loop for fresh localization data.
        odo.update();
        updateRobotPose();

        // FTC forward stick is negative, so invert it for intuitive forward motion.
        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;
        double speed = 1.0;

        utils.SetGate(NORMAL_GATE_POSITION);
       // utils.SetTurretPos(utils.TrackHiveTagPos());



        utils.drive(forward, strafe, rotate, speed);

        // Telemetry readout for both Pinpoint Pose and Turret Status
        telemetry.addLine("--- PINPOINT POSE ---");
        telemetry.addData("X / Forward (in)", "%.2f", robotX);
        telemetry.addData("Y / Strafe (in)", "%.2f", robotY);
        telemetry.addData("Heading (deg)", "%.2f", robotHeadingDeg);
        //telemetry.addData("angle to hive:", "%.2f", GetAngleDifference());

        telemetry.addLine("--- TURRET STATUS ---");
        telemetry.addData("Turret Raw Pos", "%.3f", utils.GetTurretPos());
        telemetry.addData("Turret Angle Eq (deg)", "%.2f", (utils.GetTurretPos() - 0.5) * 450.0);
        telemetry.addData("Turret velocity", utils.getTurretVelocity());
        telemetry.update();
    }

    private void updateRobotPose() {
        Pose2D pose = odo.getPosition();
        robotX = pose.getX(DistanceUnit.INCH);
        robotY = pose.getY(DistanceUnit.INCH);
        robotHeadingDeg = pose.getHeading(AngleUnit.DEGREES);
    }
    public double GetAngleDifference(double targetX, double targetY) {
        double currentTurretAngleDeg = (utils.GetTurretPos() - 0.5) * 450.0;

        double desiredTurretAngleDeg = getTurretAngleToFieldTargetDeg(targetX, targetY);

        return normalizeDegrees(desiredTurretAngleDeg - currentTurretAngleDeg);
    }
    private double calculateHiveAlignmentTurn(double driverTurn) {
        if (utils.IsAligned()) {
            return 0.0;
        }

        double tx = utils.TX();
        if (Math.abs(tx) < 0.0001) {
            return clip(driverTurn * 2.25, -MAX_ALIGN_TURN, MAX_ALIGN_TURN);
        }

        return clip(tx * ALIGN_KP, -MAX_ALIGN_TURN, MAX_ALIGN_TURN);
    }

    public double getRobotX() {
        return robotX;
    }

    public double getRobotY() {
        return robotY;
    }

    public double getRobotHeadingDeg() {
        return robotHeadingDeg;
    }


    public double getTurretAngleToFieldTargetDeg(double targetX, double targetY) {
        double fieldAngleDeg = Math.toDegrees(
                Math.atan2(targetY - robotY, targetX - robotX)
        );
        return normalizeDegrees(fieldAngleDeg - robotHeadingDeg);
    }

    private static double normalizeDegrees(double angleDeg) {
        while (angleDeg > 180.0) angleDeg -= 360.0;
        while (angleDeg <= -180.0) angleDeg += 360.0;
        return angleDeg;
    }

    private static double clip(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    @Override
    public void stop() {
        utils.drive(0.0, 0.0, 0.0, 0.0);
        utils.Setintake(0);
        utils.SetGate(NORMAL_GATE_POSITION);
    }
}