package org.firstinspires.ftc.teamcode.pedroPathing;

import android.health.connect.datatypes.units.Velocity;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Auto_ServiceHelper_WithPedroPath {
    private DcMotor Intake, MotorFeeder;
    private Limelight3A limelight;

    private DcMotorEx Turret;
    private DcMotor ServoConFront;
    private CRServo ServoCon, IntakeServo;
    private Servo HoodServo, ServoConTurret;
    public static double KP = 0.009;
    public static double KD = 0.00177; //could be a little higher?
    private final double DEAD_ZONE_DEG = 1.5;
    private double lostStartTime = -1;
    private final double LOST_DELAY = 0.5;
    double lockedPosition = 0.5;
    double servoCenter = 0.5;
    double min = 0.4;
    double max = 0.6;
    private double lastTx = 0;
    double lastError = 0;
    long lastTime = System.nanoTime();
    private ElapsedTime driveTimer = new ElapsedTime();
    private int currentpipeline = 0;

    //TODO
    public void init(HardwareMap hwMap, String autoState ) {
        MotorFeeder = hwMap.get(DcMotor.class, "motorizedtransfer");

        Intake = hwMap.get(DcMotor.class, "intake");
        Turret = hwMap.get(DcMotorEx.class, "turret");

        // TODO BELOW
        //setting PF value for flywheel turret motor
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(400, 0, 0, 15.047 );
        Turret.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        ServoConTurret = hwMap.get(Servo.class, "servo_con_turret");
        HoodServo = hwMap.get(Servo.class, "hoodservo");
        IntakeServo = hwMap.get(CRServo.class, "intakeservo");
        ServoCon = hwMap.get(CRServo.class, "servo_con_back_transfer");
        ServoConFront = hwMap.get(DcMotor.class, "servo_con_front_transfer");

        Turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorFeeder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        limelight = hwMap.get(Limelight3A.class, "limelight");

        limelight.pipelineSwitch(currentpipeline);
        limelight.start();

    }
    public double getTurretVelocity() {
        return Turret.getVelocity();
    }

    public boolean isTurretAtSpeed(double velocity) {

        return Math.abs(getTurretVelocity() - velocity) < 50;
    }

    public void StartTurret(int velocity) {
        Turret.setVelocity(velocity);
    }
    public void AutoShoot(int Velocity) {

        Intake.setPower(1.0);
        IntakeServo.setPower(-1.0);
        MotorFeeder.setPower(-1.0);
        ServoConFront.setPower(-1.0);

        Turret.setVelocity(Velocity);

    }


    public void AutoIntake() {
        Intake.setPower(1.0);
        IntakeServo.setPower(-1.0);
       // MotorFeeder.setPower(1.0);
    }
    public void AutoIntakeNear() {
            Intake.setPower(1.0);
            IntakeServo.setPower(-1.0);
            MotorFeeder.setPower(1.0);
            ServoConFront.setPower(-0.6);
    }
    public void Stop() {
        Intake.setPower(0.0);
        IntakeServo.setPower(0.0);
        MotorFeeder.setPower(0.0);
        ServoConFront.setPower(0.0);
    }
    public void lifthood() {
        HoodServo.setPosition(0.56);
    }

    public void lowerhood() {
        HoodServo.setPosition(0.9);
    }
    public void PauseTrack() {
        limelight.pause();
    }

    public boolean hasValidTarget() {
        LLResult result = limelight.getLatestResult();
        return result != null && result.isValid();
    }

    public void SetDefaultServoConTurret()
    {
        ServoConTurret.setPosition(0.5);
    }
    public void AutoTrack(int pipeline) {

        LLResult result = limelight.getLatestResult();
        limelight.pipelineSwitch(pipeline);

        long now = System.nanoTime();
        double dt = (now - lastTime) / 1e9;
        lastTime = now;

        if (dt <= 0) dt = 0.001;

        // =============================
        // NO TARGET DETECTED
        // =============================
        if (result == null || !result.isValid()) {

            // Send STOP signal (CR servo stop)
            ServoConTurret.setPosition(servoCenter);

            // Reset PD state
            lastError = 0;
            lastTime = System.nanoTime();

            return;
        }

        // =============================
        // TARGET DETECTED
        // =============================

        double tx = result.getTx();

        if (Math.abs(tx) < DEAD_ZONE_DEG) {
            tx = 0;
        }

        double error = tx;
        double derivative = (error - lastError) / dt;
        lastError = error;

        double output = (KP * error) + (KD * derivative);

        // For CR servo, output should be speed around center
        double command = servoCenter - output;

        command = Math.max(min, Math.min(max, command));

        ServoConTurret.setPosition(command);
    }

}
