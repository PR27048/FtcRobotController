package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous
public abstract class BlueFar extends LinearOpMode {
    private DcMotor FrontLeft, FrontRight, BackLeft, BackRight, Intake, Turret, MotorFeeder;

    private CRServo ServoCon, ServoConFront, IntakeServo, ServoConTurret;
    private Servo HoodServo;

    public void runOpMode(HardwareMap hwMap) {
        FrontLeft = hwMap.get(DcMotor.class, "front_left");
        FrontRight = hwMap.get(DcMotor.class, "front_right");
        BackLeft = hwMap.get(DcMotor.class, "back_left");
        BackRight = hwMap.get(DcMotor.class, "back_right");
        MotorFeeder = hwMap.get(DcMotor.class, "motorizedtransfer");

        Intake = hwMap.get(DcMotor.class, "intake");
        Turret = hwMap.get(DcMotor.class, "turret");
        ServoConTurret = hwMap.get(CRServo.class, "servo_con_turret");
        HoodServo = hwMap.get(Servo.class, "hoodservo");
        IntakeServo = hwMap.get(CRServo.class, "intakeservo");
        ServoCon = hwMap.get(CRServo.class, "servo_con_back_transfer");
        ServoConFront = hwMap.get(CRServo.class, "servo_con_front_transfer");

        FrontLeft.setDirection(DcMotor.Direction.REVERSE);
        BackLeft.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();
        TurretPower(0.6); // start turret
        // Drive
        setDrivePower(0.3); // drive

        sleep(5000); // 5 seconds

        // Stop
        setDrivePower(0); // stop drive

        TurnRobot(0.2); // facegoal
        sleep(600);
        TurnRobot(0); //stop
        sleep(2000);
        Launch(); //shoot
    }

    private void setDrivePower(double power) {
        FrontLeft.setPower(power);
        FrontRight.setPower(power);
        BackLeft.setPower(power);
        BackRight.setPower(power);
    }
    private void TurnRobot(double power) {
        FrontLeft.setPower(-power);
        FrontRight.setPower(power);
        BackLeft.setPower(-power);
        BackRight.setPower(power);
    }
    private void TurretPower(double power) {

        Turret.setPower(power);
    }

    private void Launch() {
        Intake.setPower(1.0);
        ServoConTurret.setPower(-1.0);
        MotorFeeder.setPower(0.7);
        ServoConFront.setPower(-1.0);
        IntakeServo.setPower(-1.0);

    }

}


