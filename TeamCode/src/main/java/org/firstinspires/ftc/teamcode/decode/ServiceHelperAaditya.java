package org.firstinspires.ftc.teamcode.decode;

//import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class ServiceHelperAaditya {

    private DcMotor FrontLeft, FrontRight, BackLeft, BackRight, Intake, MotorFeeder;

    private DcMotorEx Turret;
    private CRServo ServoCon, ServoConFront, IntakeServo, ServoConTurret;
    private Servo HoodServo;
    private ElapsedTime driveTimer = new ElapsedTime();

    public void init(HardwareMap hwMap, String autoState ) {
        FrontLeft = hwMap.get(DcMotor.class, "front_left");
        FrontRight = hwMap.get(DcMotor.class, "front_right");
        BackLeft = hwMap.get(DcMotor.class, "back_left");
        BackRight = hwMap.get(DcMotor.class, "back_right");
        MotorFeeder = hwMap.get(DcMotor.class, "motorizedtransfer");

        Intake = hwMap.get(DcMotor.class, "intake");
        Turret = hwMap.get(DcMotorEx.class, "turret");

        //setting PF value for flywheel turret motor
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(400, 0, 0, 14.6 );
        Turret.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        ServoConTurret = hwMap.get(CRServo.class, "servo_con_turret");
        HoodServo = hwMap.get(Servo.class, "hoodservo");
        IntakeServo = hwMap.get(CRServo.class, "intakeservo");
        ServoCon = hwMap.get(CRServo.class, "servo_con_back_transfer");
        ServoConFront = hwMap.get(CRServo.class, "servo_con_front_transfer");

        FrontRight.setDirection(DcMotor.Direction.REVERSE);
        BackRight.setDirection(DcMotor.Direction.REVERSE);

       /* switch (autoState) {
            case "FALSE":
            {
                FrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                FrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                BackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                BackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                Intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                Turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                MotorFeeder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
               // break;
           /* }
            case "TRUE":
            {*/
        FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorFeeder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //   break;
        //}
        //}
        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void drive(double forward, double strafe, double rotate, double speed) {
        double FrontLeftPower = forward - strafe - rotate;
        double FrontRightPower = forward + strafe + rotate;
        double BackLeftPower = - forward - strafe + rotate;
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

        Turret.setPower(0.41); //0.64
    }
    public void SetTurretPowerAccel(/*double TurretPower*/) {

        Turret.setPower(0.44); // 0.67
    }
    public void setFeederPower(double feederPower) {
        MotorFeeder.setPower(feederPower);
    }
    public void setServoConPower(double power) {
        ServoCon.setPower(power);
        //ServoCon.setPower(1.0);
    }

    public void setHoodAngle(double angle) {
        HoodServo.setPosition(angle);
    }

    public void setIntakeServoPower(double pow) {
        IntakeServo.setPower(pow);
    }

    public void SetServoConFrontPower(double frontPower) {
        ServoConFront.setPower(frontPower);
    }

    public void SetTurretVelocity() {

        Turret.setVelocity(1020);
        // Turret.setPower(0.47); //0.64
    }
    public boolean driveToPosition(double speed, double distance, DistanceUnit distanceUnit, double holdSeconds) {
        final double WHEEL_DIAMETER_MM = 96;
        final double ENCODER_TICKS_PER_REV = 537.7;
        final double TICKS_PER_MM = (ENCODER_TICKS_PER_REV / (WHEEL_DIAMETER_MM * Math.PI));
        final double TOLERANCE_MM = 10;
        final double TRACK_WIDTH_MM = 404;
        double targetPosition = (distanceUnit.toMm(distance) * TICKS_PER_MM);

        drive(distance, 1, 0, speed);

        FrontLeft.setTargetPosition((int) targetPosition);
        FrontRight.setTargetPosition((int) -targetPosition);
        BackLeft.setTargetPosition((int) targetPosition);
        BackRight.setTargetPosition((int) -targetPosition);

        FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if(Math.abs(targetPosition - FrontLeft.getCurrentPosition()) > (TOLERANCE_MM * TICKS_PER_MM)){
            driveTimer.reset();
        }
//telemetry.addLine("in drive to position");
        //      telemetry.addData("targetPosition", targetPosition);

        return (driveTimer.seconds() > holdSeconds);

    }
}