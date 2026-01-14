package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class AWDcode {
    private DcMotor FrontLeft, FrontRight, BackLeft, BackRight, Intake, Turret;
    private CRServo ServoCon, ServoConFront, IntakeServo, ServoConTurret;

    public void init(HardwareMap hwMap) {
        FrontLeft = hwMap.get(DcMotor.class, "front_left");
        FrontRight = hwMap.get(DcMotor.class, "front_right");
        BackLeft = hwMap.get(DcMotor.class, "back_left");
        BackRight = hwMap.get(DcMotor.class, "back_right");
        Intake = hwMap.get(DcMotor.class, "intake");
        Turret = hwMap.get(DcMotor.class, "turret");
        ServoConTurret = hwMap.get(CRServo.class, "servo_con_turret");

        IntakeServo = hwMap.get(CRServo.class, "intakeservo");
        ServoCon = hwMap.get(CRServo.class, "servo_con_back_transfer");
        ServoConFront = hwMap.get(CRServo.class, "servo_con_front_transfer");

        FrontRight.setDirection(DcMotor.Direction.REVERSE);
        BackRight.setDirection(DcMotor.Direction.REVERSE);
        FrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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

    public void aimTurret(double clockwise, double counterclockwise) {
        double ServoConTurretPower = clockwise - counterclockwise;

        double MaxTurretAimingPower = 1.0;
        double MaxTurretAimingSpeed = 1.0;

        MaxTurretAimingPower = Math.max(MaxTurretAimingPower, Math.abs(ServoConTurretPower));

        ServoConTurret.setPower((MaxTurretAimingSpeed * ServoConTurretPower / MaxTurretAimingPower));
    }

    public void SetIntakePower(double IntakePower) {

        Intake.setPower(IntakePower);
    }

    public void SetTurretPower(/*double TurretPower*/) {

        Turret.setPower(0.67);
    }

    public void setServoConPower(double power) {
        ServoCon.setPower(power);
        //ServoCon.setPower(1.0);
    }

    public void setIntakeServoPower(double pow) {
        IntakeServo.setPower(pow);
    }

    public void SetServoConFrontPower(double frontPower) {
        ServoConFront.setPower(frontPower);
    }

}