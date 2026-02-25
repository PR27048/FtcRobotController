package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class BaseHelper {

    private DcMotor FrontLeft, FrontRight, BackLeft, BackRight, Intake, MotorFeeder;
    private DcMotorEx Turret;

    private CRServo ServoCon, ServoConFront, IntakeServo;
    private Servo HoodServo, ServoConTurret;
    private Limelight3A limelight;

    private ElapsedTime driveTimer = new ElapsedTime();

    // ================= LIMELIGHT PD TUNING =================
    public static double KP = 0.009;
    public static double KD = 0.0017; //could be a little higher?
    private final double DEAD_ZONE_DEG = 1.5;
    double servoCenter = 0.5;
    double min = 0.4;
    double max = 0.6;
    private double lastTx = 0;
    double lastError = 0;
    long lastTime = System.nanoTime();
    public double turretAngle = 0.0;
    private double ServoTurns = 0.0;
    private double lastServoPos;


    //private double lastTime = 0;

    // ================= PIPELINE MENU =================
    private int currentPipeline = 0;

    // ================= INIT =================
    public void init(HardwareMap hwMap, String autoState ) {

        FrontLeft = hwMap.get(DcMotor.class, "front_left");
        FrontRight = hwMap.get(DcMotor.class, "front_right");
        BackLeft = hwMap.get(DcMotor.class, "back_left");
        BackRight = hwMap.get(DcMotor.class, "back_right");
        MotorFeeder = hwMap.get(DcMotor.class, "motorizedtransfer");

        Intake = hwMap.get(DcMotor.class, "intake");
        Turret = hwMap.get(DcMotorEx.class, "turret");
        PIDFCoefficients pidfCoefficients =
                new PIDFCoefficients(400, 0, 0, 14.6);
        Turret.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfCoefficients);


        ServoConTurret = hwMap.get(Servo.class, "servo_con_turret");
        lastServoPos = ServoConTurret.getPosition(); // initialize

        HoodServo = hwMap.get(Servo.class, "hoodservo");
        IntakeServo = hwMap.get(CRServo.class, "intakeservo");
        ServoCon = hwMap.get(CRServo.class, "servo_con_back_transfer");
        ServoConFront = hwMap.get(CRServo.class, "servo_con_front_transfer");

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
        double newPos = Math.max(0.0, Math.min(1.0, current + adjustment * 0.01));
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

            // Reset derivative so it doesn’t kick when target returns
            lastError = 0;

            return;
        }
        // Tracking Logic


        double tx = result.getTx();

        if (Math.abs(tx) < DEAD_ZONE_DEG) {
            tx = 0;
        }

        double error = tx;

        double derivative = (error - lastError) / dt;
        lastError = error;

        double output = (KP * error) + (KD * derivative);

        double targetPosition = servoCenter - output;

        // Clamp to safe servo range
        targetPosition = Math.max(min, Math.min(max, targetPosition));


        ServoConTurret.setPosition(targetPosition);
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

    public int getCurrentPipeline() {
        return currentPipeline;
    }

    public void stopLimelight() {
        limelight.stop();
    }

    // ================= OTHER FUNCTIONS =================
    public void SetIntakePower(double IntakePower) { Intake.setPower(IntakePower); }
    public void SetTurretPower() {

    }
    public void SetTurretPowerAccel() { Turret.setPower(0.44); }
    public void ReverseTurret() { Turret.setPower(-0.1); }
    public void setFeederPower(double feederPower) { MotorFeeder.setPower(feederPower); }
    public void setServoConPower(double power) { ServoCon.setPower(power); }
    public void setHoodAngle(double angle) { HoodServo.setPosition(angle); }
    public void setIntakeServoPower(double pow) { IntakeServo.setPower(pow); }
    public void SetServoConFrontPower(double frontPower) { ServoConFront.setPower(frontPower); }
    public void SetTurretVelocity() { Turret.setVelocity(1020); }
    public void SetTurretOFF() { Turret.setVelocity(0); }
    public double getTurretAngle() {
        return turretAngle;
    }


    public double getTurretPosition() {
        return ServoConTurret.getPosition();
    }
}