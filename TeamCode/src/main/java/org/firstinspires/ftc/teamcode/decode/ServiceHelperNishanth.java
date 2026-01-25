package org.firstinspires.ftc.teamcode.decode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ServiceHelperNishanth {

    private DcMotor FrontLeft, FrontRight, BackLeft, BackRight, Intake, Turret, BackFeeder;
    private CRServo ServoConFront, ServoConIntake, ServoConTurret;
    private Servo ServoConHood;

    private double hoodPosition = 0.5;
    private static final double HOOD_MIN = 0.0;
    private static final double HOOD_MAX = 1.0;
    private static final double HOOD_STEP = 0.01;

    public void init(HardwareMap hwMap) {

        FrontLeft = hwMap.get(DcMotor.class, "front_left");
        FrontRight = hwMap.get(DcMotor.class, "front_right");
        BackLeft = hwMap.get(DcMotor.class, "back_left");
        BackRight = hwMap.get(DcMotor.class, "back_right");

        Intake = hwMap.get(DcMotor.class, "intake");
        Turret = hwMap.get(DcMotor.class, "turret");
        BackFeeder = hwMap.get(DcMotor.class, "motorizedtransfer");

        ServoConFront = hwMap.get(CRServo.class, "servo_con_front_transfer");
        ServoConIntake = hwMap.get(CRServo.class, "intakeservo");
        ServoConTurret = hwMap.get(CRServo.class, "servo_con_turret");
        ServoConHood = hwMap.get(Servo.class, "hoodservo");

        FrontRight.setDirection(DcMotor.Direction.REVERSE);
        BackRight.setDirection(DcMotor.Direction.REVERSE);

        FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        ServoConHood.setPosition(hoodPosition);
    }

    public void drive(double forward, double strafe, double rotate) {
        double FrontLeftPower = forward - strafe - rotate;
        double FrontRightPower = forward + strafe + rotate;
        double BackLeftPower = -forward - strafe + rotate;
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

    public void aimTurret(double clockwise, double counterclockwise) {
        double ServoConTurretPower = -clockwise + counterclockwise;

        double MaxTurretAimingPower = 1.0;
        double MaxTurretAimingSpeed = 1.0;

        MaxTurretAimingPower = Math.max(MaxTurretAimingPower, Math.abs(ServoConTurretPower));

        ServoConTurret.setPower((MaxTurretAimingSpeed * ServoConTurretPower / MaxTurretAimingPower));
    }

    public void hoodServo(double up, double down) {

        hoodPosition += (up - down) * HOOD_STEP;
        hoodPosition = Math.max(HOOD_MIN, Math.min(HOOD_MAX, hoodPosition));
        ServoConHood.setPosition(hoodPosition);
    }

    public void SetIntakePower(double IntakePower) {
        Intake.setPower(IntakePower);
    }

    public void SetTurretPower(double TurretPower) {
        Turret.setPower(TurretPower);
    }

    public void SetBackFeederPower(double power) {
        BackFeeder.setPower(power);
    }

    public void SetServoConFrontPower(double frontPower) {
        ServoConFront.setPower(frontPower);
    }

    public void SetServoConIntakePower(double intakeServoPower) {
        ServoConIntake.setPower(intakeServoPower);
    }

    public void SetServoConHoodPower(double hoodServoPower) {
    }
}