package org.firstinspires.ftc.teamcode.Teleops;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.pedro.Constants;

public class UtilitiesEnhanced {

    private DcMotor FrontLeft, FrontRight, BackLeft, BackRight, Intake, Transfer;
    private CRServo HFeeder1, HFeeder2;
    private DcMotorEx Shooter1, Shooter2;
    private TouchSensor FireReady;

    private Servo Feeder, FrontalSweep, TurretServo;
    private ElapsedTime driveTimer = new ElapsedTime();

    private Limelight3A limelight;
    int currentPipeline = 2; // 2 = red
    private final double DEAD_ZONE_DEG = 4.0;

    // --- Pinpoint odometry & relative tag tracking state ---
    private GoBildaPinpointDriver pinpoint;

    // Baseline captured at last valid Limelight alignment (inches & radians)
    private boolean hasAlignedBaseline = false;
    private double alignedX = 0.0;
    private double alignedY = 0.0;
    private double alignedHeadingRad = 0.0;

    // Tag coordinates in the alignment-time robot frame (+X = forward, +Y = left, inches)
    private double alignedTagRelX = 0.0;
    private double alignedTagRelY = 0.0;

    public static class RelativeTagEstimate {
        public final boolean valid;
        public final boolean fromFreshLimelight;

        // Movement since last Limelight alignment (in alignment robot frame, inches / deg)
        public final double movedForward;
        public final double movedLeft;
        public final double movedHeadingDeg;

        // Tag position relative to CURRENT robot pose & facing (+X = forward, +Y = left)
        public final double tagRelForward;
        public final double tagRelLeft;
        public final double tagDistance;
        public final double tagBearingDeg; // positive = left of current chassis forward, negative = right

        public RelativeTagEstimate(
                boolean valid,
                boolean fromFreshLimelight,
                double movedForward,
                double movedLeft,
                double movedHeadingDeg,
                double tagRelForward,
                double tagRelLeft,
                double tagDistance,
                double tagBearingDeg) {
            this.valid = valid;
            this.fromFreshLimelight = fromFreshLimelight;
            this.movedForward = movedForward;
            this.movedLeft = movedLeft;
            this.movedHeadingDeg = movedHeadingDeg;
            this.tagRelForward = tagRelForward;
            this.tagRelLeft = tagRelLeft;
            this.tagDistance = tagDistance;
            this.tagBearingDeg = tagBearingDeg;
        }
    }

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
        //-----------------------------------------------------------

        Feeder = hwMap.get(Servo.class, "Feeder");
        FrontalSweep = hwMap.get(Servo.class, "FrontalSweep");
        // TurretServo = hwMap.get(Servo.class, "TurretServo");

        //-----------------------------------------------------------
        HFeeder1 = hwMap.get(CRServo.class, "Hfeeder1");
        HFeeder2 = hwMap.get(CRServo.class, "Hfeeder2");
        //-----------------------------------------------------------

        limelight = hwMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(currentPipeline);
        limelight.start();
        //-----------------------------------------------------------

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

        // Initialize Pinpoint odometry from Constants
        initOdometry(hwMap);

        driveTimer.reset();
    }

    /**
     * Configures the goBILDA Pinpoint driver from Constants.localizerConfig.
     */
    public void initOdometry(HardwareMap hwMap) {
        pinpoint = hwMap.get(GoBildaPinpointDriver.class, Constants.localizerConfig.name.get());
        pinpoint.setOffsets(
                Constants.localizerConfig.xPodOffset.get(),
                Constants.localizerConfig.yPodOffset.get(),
                Constants.localizerConfig.offsetUnits.get()
        );
        pinpoint.setEncoderResolution(Constants.localizerConfig.podType.get());
        pinpoint.setEncoderDirections(
                Constants.localizerConfig.xPodDirection.get(),
                Constants.localizerConfig.yPodDirection.get()
        );
        pinpoint.resetPosAndIMU();
        pinpoint.update();

        hasAlignedBaseline = false;
    }

    /**
     * Updates Pinpoint odometry and computes:
     * 1) How far the robot has moved since the last valid Limelight alignment
     * 2) Where the tag is relative to where the robot is currently facing
     */
    public RelativeTagEstimate updateRelativeTagTracking() {
        if (pinpoint == null) {
            return new RelativeTagEstimate(
                    false, false, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
            );}

        pinpoint.update();
        Pose2D currentPose = pinpoint.getPosition();

        double currX = currentPose.getX(DistanceUnit.INCH);
        double currY = currentPose.getY(DistanceUnit.INCH);
        double currHeadingRad = AngleUnit.normalizeRadians(
                currentPose.getHeading(AngleUnit.RADIANS)
        );

        // Refresh alignment baseline whenever Limelight has a valid lock
        LLResult result = (limelight != null) ? limelight.getLatestResult() : null;
        boolean freshLock = (IsAligned());

        if (freshLock) {
            double rangeIn = GetDistance();
            // Limelight tx positive is right; convert to +Y-left bearing
            double bearingRad = -Math.toRadians(result.getTx());

            alignedX = currX;
            alignedY = currY;
            alignedHeadingRad = currHeadingRad;

            alignedTagRelX = rangeIn * Math.cos(bearingRad);
            alignedTagRelY = rangeIn * Math.sin(bearingRad);
            hasAlignedBaseline = true;
        }

        if (!hasAlignedBaseline) {
            return new RelativeTagEstimate(
                    false, false, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
            );
        }

        // Robot displacement since alignment, expressed in alignment-time robot coordinates
        double globalDx = currX - alignedX;
        double globalDy = currY - alignedY;

        double cosAlign = Math.cos(-alignedHeadingRad);
        double sinAlign = Math.sin(-alignedHeadingRad);

        double movedForward = globalDx * cosAlign - globalDy * sinAlign;
        double movedLeft = -(globalDx * sinAlign + globalDy * cosAlign);
        double deltaHeadingRad = AngleUnit.normalizeRadians(currHeadingRad - alignedHeadingRad);

        // Vector from current robot location to saved tag location (in alignment frame)
        double toTagX = alignedTagRelX - movedForward;
        double toTagY = alignedTagRelY - movedLeft;

        // Rotate into CURRENT robot-facing frame (+X = forward, +Y = left)
        double cosTurn = Math.cos(-deltaHeadingRad);
        double sinTurn = Math.sin(-deltaHeadingRad);

        double tagRelForward = toTagX * cosTurn - toTagY * sinTurn;
        double tagRelLeft = toTagX * sinTurn + toTagY * cosTurn;

        double tagDistance = Math.hypot(tagRelForward, tagRelLeft);
        double tagBearingDeg = Math.toDegrees(Math.atan2(tagRelLeft, tagRelForward));

        return new RelativeTagEstimate(
                true,
                freshLock,
                movedForward,
                movedLeft,
                Math.toDegrees(deltaHeadingRad),
                tagRelForward,
                tagRelLeft,
                tagDistance,
                tagBearingDeg
        );
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

    public void Setintake(double Intakespeed, double Transferspeed) {
        Intake.setPower(Intakespeed);
        Transfer.setPower(-Transferspeed);
    }

    public boolean SlingerUp() {
        return FireReady.isPressed();
    }

    public boolean HiveUp() {
        LLResult result = (limelight != null) ? limelight.getLatestResult() : null;
        return result != null && result.isValid();
    }

    public boolean IsAligned() {
        LLResult result = (limelight != null) ? limelight.getLatestResult() : null;
        if (result == null || !result.isValid()) {
            return false;
        }
        double tx = result.getTx();
        return Math.abs(tx) < DEAD_ZONE_DEG;
    }

    public double TX() {
        LLResult result = (limelight != null) ? limelight.getLatestResult() : null;
        if (result == null || !result.isValid()) {
            return 0.0;
        }
        return result.getTx();
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
            if (limelight != null) {
                limelight.pipelineSwitch(currentPipeline);
            }
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

    public double GetDistance() {
        LLResult result = (limelight != null) ? limelight.getLatestResult() : null;
        if (result == null || !result.isValid()) {
            return 0.0;
        }

        double h1 = 6.5; // Camera height in inches
        double h2 = 50.25; // Target AprilTag height in inches
        double a1 = Math.toRadians(63.43); // Camera mounting angle in radians

        double ty = result.getTy();
        double a2 = Math.toRadians(ty);

        return (h2 - h1) / Math.tan(a1 + a2);
    }

    public double yawerror() {
        LLResult result = (limelight != null) ? limelight.getLatestResult() : null;

        if (result != null && result.isValid()) {
            Pose3D botpose = result.getBotpose();
            if (botpose != null) {
                return botpose.getOrientation().getYaw();
            }
        }
        return 0.0;
    }

    public void SetTurretPos(double pos) {
        if (TurretServo != null) {
            TurretServo.setPosition(pos);
        }
    }

    public double GetTurretPos() {
        return (TurretServo != null) ? TurretServo.getPosition() : 0.5;
    }

    public boolean isTurretAtSpeed(double velocity) {
        return Math.abs(getTurretVelocity() - velocity) < 40;
    }

    public double getTurretVelocity() {
        return Shooter1.getVelocity();
    }

    public void SetFeeder(double power) {
        HFeeder1.setPower(power);
        HFeeder2.setPower(-power);
    }

    public void SetGate(double pos) {
        Feeder.setPosition(pos);
    }

    public void SetSweep(double pos) {
        FrontalSweep.setPosition(pos);
    }

    public void SetShooterVelocity(double velocity) {
        Shooter1.setVelocity(velocity);
        Shooter2.setVelocity(velocity);
    }

} // End of class