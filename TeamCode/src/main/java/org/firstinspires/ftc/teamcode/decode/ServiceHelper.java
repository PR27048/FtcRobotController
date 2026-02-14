package org.firstinspires.ftc.teamcode.decode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class ServiceHelper {
    private DcMotor FrontLeft, FrontRight, BackLeft, BackRight, Intake,  MotorFeeder;
    private DcMotorEx Turret;
    private CRServo ServoConFront, IntakeServo, ServoConTurret;
    private Servo HoodServo;
    private ElapsedTime driveTimer = new ElapsedTime();

    public void init(HardwareMap hwMap) {
        FrontLeft =hwMap.get(DcMotor .class,"front_left");
        FrontRight =hwMap.get(DcMotor .class,"front_right");
        BackLeft =hwMap.get(DcMotor.class,"back_left");
        BackRight =hwMap.get(DcMotor.class,"back_right");

        //intake and intake feeder and flywheel
        Intake =hwMap.get(DcMotor.class,"intake");
        MotorFeeder =hwMap.get(DcMotor.class,"motorizedtransfer");
        Turret =hwMap.get(DcMotorEx.class,"turret");

        //setting PF value for flywheel turret motor
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(400, 0, 0, 14.6 );
        Turret.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        //initialize servos
        IntakeServo =hwMap.get(CRServo .class,"intakeservo");
        ServoConFront =hwMap.get(CRServo .class,"servo_con_front_transfer");
        ServoConTurret =hwMap.get(CRServo .class,"servo_con_turret");
        HoodServo =hwMap.get(Servo .class,"hoodservo");

        //reverse wheel position
        FrontRight.setDirection(DcMotor.Direction.REVERSE);
        BackRight.setDirection(DcMotor.Direction.REVERSE);

        FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //Turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorFeeder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       // MotorFeeder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
     //   Turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void TurretEncoder() {
        Turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    public void drive(double forward, double strafe, double rotate) {
        double FrontLeftPower = forward - strafe - rotate;
        double FrontRightPower = forward + strafe + rotate;
        double BackLeftPower = - forward - strafe + rotate;
        double BackRightPower = forward - strafe + rotate;

        double maxPower = 1.0;
        double maxSpeed = 1.0;

        maxPower = Math.max(maxPower, Math.abs(FrontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(FrontRightPower));
        maxPower = Math.max(maxPower, Math.abs(BackLeftPower));
        maxPower = Math.max(maxPower, Math.abs(BackRightPower));

        FrontLeft.setPower(maxSpeed * FrontLeftPower / maxPower);
        FrontRight.setPower(maxSpeed * FrontRightPower / maxPower);
        BackLeft.setPower(maxSpeed * BackLeftPower / maxPower);
        BackRight.setPower(maxSpeed * BackRightPower / maxPower);
    }

    //intake motor
    public void SetIntakePower(double IntakePower) {

        Intake.setPower(IntakePower);
    }
    //intake servo power
    public void setIntakeServoPower(double power) {
        IntakeServo.setPower(power);
    }

    public void SetServoConFrontPower(double frontPower) {
        ServoConFront.setPower(frontPower);
    }
    public void SetTurretPower() {

        Turret.setPower(0.42);
        // Turret.setPower(0.47); //0.64
    }

    public void SetTurretVelocity() {

        Turret.setVelocity(1020);
        // Turret.setPower(0.47); //0.64
    }

    public void setAutoTurret(double power) {
        Turret.setPower(power);

    }
    public void TurretAccel() {
        Turret.setPower(0.44);
    }
    public void setFeederPower(double feederPower) {
        MotorFeeder.setPower(feederPower);
    }

    public void setHoodAngle(double angle) {
        HoodServo.setPosition(angle);
    }
    public void aimTurret(double clockwise, double counterclockwise) {
        double ServoConTurretPower = clockwise - counterclockwise;

        double MaxTurretAimingPower = 1.0;
        double MaxTurretAimingSpeed = 1.0;

        MaxTurretAimingPower = Math.max(MaxTurretAimingPower, Math.abs(ServoConTurretPower));

        ServoConTurret.setPower((MaxTurretAimingSpeed * ServoConTurretPower / MaxTurretAimingPower));
    }

    public void AutoLaunch() {
        Intake.setPower(1.0);
        //ServoConTurret.setPower(-1.0);
        MotorFeeder.setPower(-0.7);
        ServoConFront.setPower(-1.0);
        IntakeServo.setPower(-1.0);
    }
    public void AutoIntake() {
        Intake.setPower(1.0);
        //ServoConTurret.setPower(-1.0);
        MotorFeeder.setPower(0.7);
        ServoConFront.setPower(-1.0);
        IntakeServo.setPower(-1.0);
    }

    public void AutoEndStop() {
        Turret.setPower(0);
        ServoConFront.setPower(0);
        MotorFeeder.setPower(0);
        IntakeServo.setPower(0);
        Intake.setPower(0);
        //drive(0.0, 0.0, 0.0);
    }
}
