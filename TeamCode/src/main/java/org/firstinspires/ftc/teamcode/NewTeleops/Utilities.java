package org.firstinspires.ftc.teamcode.NewTeleops;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.pedropathing.math.Pose;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.Range;

import com.qualcomm.robotcore.hardware.TouchSensor;

public class Utilities {

    private DcMotor FrontLeft, FrontRight, BackLeft, BackRight, Intake, Transfer;
    private CRServo HFeeder1, HFeeder2;
    private DcMotorEx Shooter1, Shooter2;
    private TouchSensor FireReady;

    private Servo Feeder, FrontalSweep, TurretServo;
    private ElapsedTime driveTimer = new ElapsedTime();

    private Limelight3A limelight;
    int currentPipeline = 2; // 2 = red
    private final double DEAD_ZONE_DEG = 4.0;

    private Pose anchorPose = null;
    private double anchorDistanceInches = 72.0; // Fallback starting distance

    // GEAR RATIO: 1:4 ratio means the servo turns 4 times for every 1 turret turn
    private final double GEAR_RATIO = 4.0;

    // goBILDA 5-turn servo total physical range = 5 turns * 360° = 1800°
    private final double MAX_SERVO_DEGREES = 1800.0;

    // Stores the last commanded angle internally for open-loop servo baseline tracking
    private double lastTargetAngle = 0.0;

    public void init(HardwareMap hwMap, String autoState) {
        FrontLeft = hwMap.get(DcMotor.class, "front_left");
        FrontRight = hwMap.get(DcMotor.class, "front_right");
        BackLeft = hwMap.get(DcMotor.class, "back_left");
        BackRight = hwMap.get(DcMotor.class, "back_right");
        Intake = hwMap.get(DcMotor.class, "Intake");
        Transfer = hwMap.get(DcMotor.class, "Transfer");

        Shooter1 = hwMap.get(DcMotorEx.class, "Shooter1");
        Shooter2 = hwMap.get(DcMotorEx.class, "Shooter2");

        FireReady = hwMap.get(TouchSensor.class, "FireReady");

        PIDFCoefficients pidfCoefficients =
                new PIDFCoefficients(83, 0, 0, 13.699);  // F 13.699 p: 83
        Shooter1.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        Shooter2.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        Shooter1.setDirection(DcMotor.Direction.REVERSE);
        Shooter2.setDirection(DcMotor.Direction.REVERSE);
        // -----------------------------------------------------------

        Feeder = hwMap.get(Servo.class, "Feeder");
        FrontalSweep = hwMap.get(Servo.class, "FrontalSweep");
        TurretServo = hwMap.get(Servo.class, "TurretServo");

        // -----------------------------------------------------------
        HFeeder1 = hwMap.get(CRServo.class, "Hfeeder1");
        HFeeder2 = hwMap.get(CRServo.class, "Hfeeder2");
        // -----------------------------------------------------------

        limelight = hwMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(currentPipeline);
        limelight.start();
        // -----------------------------------------------------------

        FrontRight.setDirection(DcMotor.Direction.REVERSE);
        BackRight.setDirection(DcMotor.Direction.REVERSE);
        FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Shooter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        Shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        driveTimer.reset();
    }

    public void drive(double forward, double strafe, double rotate, double speed) {
        double FrontLeftPower = forward - strafe - rotate;
        double FrontRightPower = forward + strafe + rotate;
        double BackLeftPower = forward + strafe - rotate;
        double BackRightPower = forward - strafe + rotate;

        double maxPower = speed;
        double maxSpeed = speed;

        maxPower = Math.max(maxPower, Math.abs(FrontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(FrontRightPower));
        maxPower = Math.max(maxPower, Math.abs(BackLeftPower));
        maxPower = Math.max(maxPower, Math.abs(BackRightPower));

        FrontLeft.setPower(maxSpeed * FrontLeftPower / maxPower);
        FrontRight.setPower(maxSpeed * FrontRightPower / maxPower);
        BackLeft.setPower(maxSpeed * BackLeftPower / maxPower);
        BackRight.setPower(maxSpeed * BackRightPower / maxPower);
    }


    public void updateTurret(Pose currentPose) {
        LLResult result = getLimelightResult();
        double targetAngle = calculateServoTarget(currentPose, result, lastTargetAngle);
        lastTargetAngle = targetAngle;
        setTurretPositionDegrees(targetAngle);
    }

    public LLResult getLimelightResult() {
        return limelight.getLatestResult();
    }

    public double calculateServoTarget(Pose currentPose, LLResult result, double currentTurretAngle) {
        boolean hasTarget = (result != null && result.isValid());

        if (hasTarget) {
            double measuredDistance = getDistance();
            if (measuredDistance > 0) {
                this.anchorDistanceInches = measuredDistance;
            }
            this.anchorPose = currentPose;
        }

        // If no anchor has been set yet since boot, hold center
        if (anchorPose == null) {
            return 0.0;
        }

        double targetTurretAngleDeg = 0.0;

        if (hasTarget) {
            targetTurretAngleDeg = currentTurretAngle + result.getTx();
        } else {
            // 2B. ODOMETRY BLIND TRACKING: Use Pedro Pathing's x(), y(), and heading() methods
            double dxWorld = currentPose.x() - anchorPose.x();
            double dyWorld = currentPose.y() - anchorPose.y();
            double headingDelta = currentPose.heading() - anchorPose.heading();

            // Transform world delta into robot-local anchor frame
            double localForward = dxWorld * Math.cos(-anchorPose.heading()) - dyWorld * Math.sin(-anchorPose.heading());
            double localStrafe  = dxWorld * Math.sin(-anchorPose.heading()) + dyWorld * Math.cos(-anchorPose.heading());

            // Compute relative vector to target
            double targetX = anchorDistanceInches - localForward;
            double targetY = -localStrafe;

            targetTurretAngleDeg = Math.toDegrees(Math.atan2(targetY, targetX)) - Math.toDegrees(headingDelta);
        }

        // 3. GEAR RATIO MULTIPLIER: Scale turret angle up by 4.0 for your 1:4 servo setup
        double targetServoAngleDeg = targetTurretAngleDeg * GEAR_RATIO;

        // 4. CLAMP TO 5-TURN LIMITS: Max +/- 900° from center (1800° total range)
        double maxServoLimit = MAX_SERVO_DEGREES / 2.0;
        return Math.max(-maxServoLimit, Math.min(maxServoLimit, targetServoAngleDeg));
    }

    public void setTurretPositionDegrees(double targetServoAngleDeg) {
        // Map 1800° range (-900° to +900°) to standard servo position (0.0 to 1.0)
        double maxServoLimit = MAX_SERVO_DEGREES / 2.0;
        double clampedAngle = Math.max(-maxServoLimit, Math.min(maxServoLimit, targetServoAngleDeg));
        double servoPosition = (clampedAngle + maxServoLimit) / MAX_SERVO_DEGREES;
        TurretServo.setPosition(Range.clip(servoPosition, 0.0, 1.0));
    }

    public void Setintake(double Intakespeed, double Transferspeed) {
        Intake.setPower(Intakespeed);
        Transfer.setPower(-Transferspeed);
    }

    public void updatePipelineMenu(boolean dpadUp, boolean dpadDown) {
        int selectedPipeline = currentPipeline;

        if (dpadUp) {
            selectedPipeline = 2;
        } else if (dpadDown) {
            selectedPipeline = 3;
        }

        if (selectedPipeline != currentPipeline) {
            currentPipeline = selectedPipeline;
            limelight.pipelineSwitch(currentPipeline);
        }
    }

    public int getCurrentPipeline() {
        if (currentPipeline == 2) {
            return 2;
        } else if (currentPipeline == 3) {
            return 3;
        }
        return -1;
    }

    public void TrackTag() {
        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) {
            return;
        }

        double targetAngle = calculateServoTarget(anchorPose, result, 0.0);
        setTurretPositionDegrees(targetAngle);
    }

    public double getDistance() {

        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) {
            return -1; // No valid target
        }

        double ty = result.getTy();

        double cameraHeight = 16.689; // inches
        double tagHeight = 50.25;    // inches
        double cameraAngle = 35;    // degrees

        double angle = cameraAngle + ty;

        if (Math.abs(angle) < 0.1) {
            return -1; // Prevent divide-by-zero
        }

        return (tagHeight - cameraHeight) /
                Math.tan(Math.toRadians(angle));
    }

    public double getTurretVelocity() {
        return Shooter1.getVelocity();
    }

    public void SetShooterVelocity(double velocity) {
        Shooter1.setVelocity(velocity);
        Shooter2.setVelocity(velocity);
    }

} // End of class