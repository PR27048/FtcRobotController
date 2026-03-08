package org.firstinspires.ftc.teamcode.decode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
//import com.qualcomm.hardware.bhi260.BHI260IMU;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

// IMU angle representation
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class FullFieldHelper {
    private BHI260IMU imu;

    private DcMotor FrontLeft, FrontRight, BackLeft, BackRight, Intake, MotorFeeder, ServoConFront;
    private DcMotorEx Turret;

    private CRServo ServoCon, IntakeServo;
    private Servo HoodServo, ServoConTurret;
    private Limelight3A limelight;


    private ElapsedTime driveTimer = new ElapsedTime();

    // ================= LIMELIGHT PD TUNING =================
    public static double KP = 0.009;
    public static double KD = 0.00177; //could be a little higher?
    private final double DEAD_ZONE_DEG = 1.5;
    private double lostStartTime = -1;
    private final double LOST_DELAY = 0.5;
    double lockedPosition = 0.5;
    double servoCenter = 0.5;
    boolean Losttarget = false;
    double min = 0.4;
    double OFFSETRED = -0.018;
    double OFFSETBLUE = -0.018;
    double max = 0.6;
    private double lastTx = 0;
    double lastError = 0;
    long lastTime = System.nanoTime();

    //private double lastTime = 0;

    // ================= PIPELINE MENU =================
    int currentPipeline = 0;

    // ================= INIT =================
    public void init(HardwareMap hwMap, String autoState) {

        FrontLeft = hwMap.get(DcMotor.class, "front_left");
        FrontRight = hwMap.get(DcMotor.class, "front_right");
        BackLeft = hwMap.get(DcMotor.class, "back_left");
        BackRight = hwMap.get(DcMotor.class, "back_right");
        MotorFeeder = hwMap.get(DcMotor.class, "motorizedtransfer");
        ServoConFront = hwMap.get(DcMotor.class, "servo_con_front_transfer");

        Intake = hwMap.get(DcMotor.class, "intake");
        Turret = hwMap.get(DcMotorEx.class, "turret");
        PIDFCoefficients pidfCoefficients =
                new PIDFCoefficients(400, 0, 0, 15.047);
        Turret.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfCoefficients);


        ServoConTurret = hwMap.get(Servo.class, "servo_con_turret");
        HoodServo = hwMap.get(Servo.class, "hoodservo");
        IntakeServo = hwMap.get(CRServo.class, "intakeservo");
        ServoCon = hwMap.get(CRServo.class, "servo_con_back_transfer");

        limelight = hwMap.get(Limelight3A.class, "limelight");

        limelight.pipelineSwitch(currentPipeline);
        limelight.start();

        FrontRight.setDirection(DcMotor.Direction.REVERSE);
        BackRight.setDirection(DcMotor.Direction.REVERSE);

        FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorFeeder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        driveTimer.reset();


       // imu = hwMap.get(BHI260IMU.class, "imu");  // MUST match config name exactly

        // Create a parameters object
        /*BHI260IMU.Parameters parameters = new BHI260IMU.Parameters();

// Set desired parameters
        parameters.angleUnit = BHI260IMU.Parameters.AngleUnit.RADIANS; // Ensure AngleUnit is accessed correctly
        parameters.loggingEnabled = false;

// Initialize the IMU with the parameters
        imu.initialize(parameters);*/
        //starting hoodservo position, hood MUST be fully lowered with this servo position
        HoodServo.setPosition(1.0);

    }

    // ================= DRIVE =================
    public void drive(double forward, double strafe, double rotate, double speed) {

        double FrontLeftPower = forward - strafe - rotate;
        double FrontRightPower = forward + strafe + rotate;
        double BackLeftPower = -forward - strafe + rotate;
        double BackRightPower = forward - strafe + rotate;

        double maxPower = Math.max(1.0,
                Math.max(Math.abs(FrontLeftPower),
                        Math.max(Math.abs(FrontRightPower),
                                Math.max(Math.abs(BackLeftPower),
                                        Math.abs(BackRightPower)))));

        FrontLeft.setPower(speed * FrontLeftPower / maxPower);
        FrontRight.setPower(speed * FrontRightPower / maxPower);
        BackLeft.setPower(speed * BackLeftPower / maxPower);
        BackRight.setPower(speed * BackRightPower / maxPower);
    }

    // ================= MANUAL TURRET =================
    public void aimTurret(double clockwise, double counterclockwise) {
        double current = ServoConTurret.getPosition();
        double adjustment = clockwise - counterclockwise;
        double newPos = Math.max(0.0, Math.min(1.0, current + adjustment * 0.2));
        ServoConTurret.setPosition(newPos);
    }

    // ================= APRILTAG TRACKING =================



    public void trackWithLimelight() {

        LLResult result = limelight.getLatestResult();

        long now = System.nanoTime();
        double dt = (now - lastTime) / 1e9;   // seconds
        lastTime = now;

        // Prevent divide-by-zero or crazy derivative spike
        if (dt <= 0) dt = 0.001;

        //no target detected then:
        if (result == null || !result.isValid()) {
            Losttarget = true;
            if (lostStartTime < 0) {
                lostStartTime = System.nanoTime() / 1e9;
            }

            double currentTime = System.nanoTime() / 1e9;
            double lostDuration = currentTime - lostStartTime;

            if (lostDuration > LOST_DELAY) {
                // Smoothly return to center
                //double currentPos = ServoConTurret.getPosition();
                //double newPos = currentPos + (servoCenter - currentPos) * 0.05;
                //newPos = Math.max(min, Math.min(max, newPos));
                ServoConTurret.setPosition(servoCenter);
                lastError = 0;
                lastTime = System.nanoTime();
                //gamepad2.rumble(500);
            }

            return;
        } else {
            lostStartTime = -1; // Reset timer when target found
            //limelight.start();
            Losttarget = false;
        }

        // Tracking Logic


        double tx = result.getTx();

        if (Math.abs(tx) < DEAD_ZONE_DEG) {
            tx = 0;
        }

        double error = tx;
        double OFFSET = -0.018;
        double derivative = (error - lastError) / dt;
        if (currentPipeline == 0) {
            OFFSET = OFFSETRED;
        } else if (currentPipeline == 1) {
            OFFSET = OFFSETBLUE;
        }

        // derivative = Math.max(-50, Math.min(50, derivative)); //CAN ADD IF WANTED BUT MUST TUNE LATER AGAIN

        lastError = error;

        double output = (KP * error) + (KD * derivative);

        double targetPosition = servoCenter - output;

        // Clamp to safe servo range
        targetPosition = Math.max(min, Math.min(max, targetPosition));

        /*  Limelight offset???? check later
        if (tx<DEAD_ZONE_DEG) {
            targetPosition+=0.1;
        } else if (tx>DEAD_ZONE_DEG) {
            targetPosition-=0.1;

        }

         */
        ServoConTurret.setPosition(targetPosition+OFFSET);
    }

    /*public boolean AlignedWithTag() {
        LLResult result = limelight.getLatestResult();
        double tx = result.getTx();
        if (!result.isValid()) return false;

        return Math.abs(tx) <= 3.5;
    }*/


    public boolean LostTag() {
        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) {
            return true;
        } else {
            return false;
        }

    }




    // ================= PIPELINE MENU (DPAD SELECT) =================
    public void updatePipelineMenu(boolean dpadUp, boolean dpadDown) {

        int selectedPipeline = currentPipeline;

        if (dpadUp) {
            selectedPipeline = 0;
        }
        else if (dpadDown) {
            selectedPipeline = 1;
        }

        if (selectedPipeline != currentPipeline) {
            currentPipeline = selectedPipeline;
            limelight.pipelineSwitch(currentPipeline);
        }
    }
    public void setlimelightpipeline() {
        limelight.pipelineSwitch(currentPipeline);
    }
    public int getCurrentPipeline() {
        if (currentPipeline == 0) {
            return 0;
        } else if (currentPipeline == 1) {
            return 1;
        }
        return -1;
    }

    public void stopLimelight() {
        limelight.stop();
    }

    // ================= OTHER FUNCTIONS =================
    public void SetIntakePower(double IntakePower) { Intake.setPower(IntakePower); }
    public double getTurretVelocity() {
        return Turret.getVelocity();
    }
    public double getTX() {
        // Get latest Limelight result
        LLResult result = limelight.getLatestResult();

        if (result != null /*&& result.isValid()*/) {
            return result.getTx(); // horizontal offset in degrees
        } else {
            return 0.0; // fallback if no tag detected
        }
    }
    public double getHeading() {
    /*    if (imu != null && imu.isGyroCalibrated()) {
            Orientation o = imu.getAngularOrientation(
                    AxesReference.INTRINSIC,
                    AxesOrder.ZYX,
                    AngleUnit.RADIANS
            );
            return o.firstAngle;
        }*/
        return 0.0;
    }




    public double getDistance() {
        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) {
            return -1; // No valid target
        }

        double ty = result.getTy();

        double cameraHeight = 14; // inches
        double tagHeight = 29.5;    // inches
        double cameraAngle = 23.5;    // degrees

        double angle = cameraAngle + ty;

        if (Math.abs(angle) < 0.1) {
            return -1; // Prevent divide-by-zero
        }

        return (tagHeight - cameraHeight) /
                Math.tan(Math.toRadians(angle));
    }


    public boolean isTurretAtSpeed(double velocity) {

        return Math.abs(getTurretVelocity() - velocity) < 40;
    }
    public void setHood(double pos) {
        HoodServo.setPosition(pos);
    }
    public void SetTurretPowerAccel() { Turret.setPower(0.44); }
    public void ReverseTurret() { Turret.setPower(-0.1); }
    public void setFeederPower(double feederPower) { MotorFeeder.setPower(feederPower); }
    public void setServoConPower(double power) { ServoCon.setPower(power); }
    public void setHoodAngle(double angle) { HoodServo.setPosition(angle); }
    public void setIntakeServoPower(double pow) { IntakeServo.setPower(pow); }
    public void SetServoConFrontPower(double frontPower) { ServoConFront.setPower(frontPower); }



    public void SetTurretVelocity(double velocity) {
        Turret.setVelocity(velocity);
    }



    public void SetTurretOFF() { Turret.setVelocity(0); }


    public double getTurretPosition() {
        return ServoConTurret.getPosition();
    }
    public double getHoodPosition() {
        return HoodServo.getPosition();
    }
}
