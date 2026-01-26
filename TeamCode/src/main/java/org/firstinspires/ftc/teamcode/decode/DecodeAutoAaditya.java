package org.firstinspires.ftc.teamcode.decode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@Autonomous(name = "DecodeAutoAaditya", group = "Auto")
public class DecodeAutoAaditya extends LinearOpMode {
    // Motors
    private DcMotor FrontLeft, FrontRight, BackLeft, BackRight;
    private DcMotor Intake, Turret, MotorFeeder;
    // Servos
    private CRServo ServoCon, ServoConFront, IntakeServo, ServoConTurret;
    private Servo HoodServo;

    private void initHardware() {
        FrontLeft = hardwareMap.get(DcMotor.class, "front_left");
        FrontRight = hardwareMap.get(DcMotor.class, "front_right");
        BackLeft = hardwareMap.get(DcMotor.class, "back_left");
        BackRight = hardwareMap.get(DcMotor.class, "back_right");

        MotorFeeder = hardwareMap.get(DcMotor.class, "motorizedtransfer");
        Intake = hardwareMap.get(DcMotor.class, "intake");
        Turret = hardwareMap.get(DcMotor.class, "turret");

        ServoConTurret = hardwareMap.get(CRServo.class, "servo_con_turret");
        IntakeServo = hardwareMap.get(CRServo.class, "intakeservo");
      //  ServoCon = hardwareMap.get(CRServo.class, "servo_con_back_transfer");
        ServoConFront = hardwareMap.get(CRServo.class, "servo_con_front_transfer");
        HoodServo = hardwareMap.get(Servo.class, "hoodservo");

        FrontLeft.setDirection(DcMotor.Direction.REVERSE);
        BackLeft.setDirection(DcMotor.Direction.REVERSE);

        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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

    // Define your auto modes
    enum AutoMode {
        BLUE_NEAR,
        BLUE_FAR,

        RED_NEAR,

        RED_FAR
    }

    AutoMode selectedMode = AutoMode.BLUE_NEAR;

    // Button state tracking (prevents rapid toggling)
    boolean lastLeft = false;
    boolean lastRight = false;
    boolean lastUp = false;

    boolean lastDown = false;

    boolean lastGuide = false;
    /*private void setDrivePower(double power) {
        FrontLeft.setPower(power);
        FrontRight.setPower(power);
        BackLeft.setPower(power);
        BackRight.setPower(power);
    }*/

    private void TurretPower() {
        Turret.setPower(0.5);
    }

    private void Launch() {
        Intake.setPower(1.0);
        //ServoConTurret.setPower(-1.0);
        MotorFeeder.setPower(-0.7);
        ServoConFront.setPower(-1.0);
        IntakeServo.setPower(-1.0);
    }
    private void Intake() {
        Intake.setPower(1.0);
        //ServoConTurret.setPower(-1.0);
        MotorFeeder.setPower(0.7);
        ServoConFront.setPower(-1.0);
        IntakeServo.setPower(-1.0);
    }
    /*private void TurnRobot(double power) {
        FrontLeft.setPower(power);
        FrontRight.setPower(-power);
        BackLeft.setPower(power);
        BackRight.setPower(-power);
    }*/
    @Override
    public void runOpMode() {
        initHardware();

//pre menu
        while (!isStarted() && !isStopRequested()) {

            // Navigate options
            if (gamepad1.dpad_right && !lastRight) {
                selectedMode = AutoMode.BLUE_FAR;
            }

            if (gamepad1.dpad_left && !lastLeft) {
                selectedMode = AutoMode.BLUE_NEAR;
            }
            if (gamepad1.dpad_up && !lastUp) {
                selectedMode = AutoMode.RED_FAR;
            }
            if (gamepad1.dpad_down && !lastDown) {
                selectedMode = AutoMode.RED_NEAR;
            }

            // Confirm selection with Logitech center button
          /*  if (gamepad1.guide && !lastGuide) {
                confirmed = true;
            }*/

            telemetry.addLine("Select Auto");
            //telemetry.addData("Confirmed", confirmed);
            telemetry.addLine("D-Pad to select auto \n right:BLUE_FAR-C1 \n left:BLUE_NEAR-A5 \n up:RED_FAR-D1 \n down:RED_NEAR-E5");
            telemetry.addData("\n Selected Auto", selectedMode);
            //  telemetry.addLine("Center (Logitech) Button: Confirm");
            telemetry.addLine("Press START when ready");
            telemetry.update();

            lastLeft = gamepad1.dpad_left;
            lastRight = gamepad1.dpad_right;
            lastUp = gamepad1.dpad_up;
            lastDown = gamepad1.dpad_down;
            lastGuide = gamepad1.guide;

            sleep(20);
        }

        waitForStart();

        if (isStopRequested()) return;

        switch (selectedMode) {
            case BLUE_NEAR:
                runBlueClose();
                break;

            case BLUE_FAR:
                runBlueFar();
                break;

            case RED_FAR:
                runRedFar();
                break;

            case RED_NEAR:
                runRedClose();
                break;
        }
    }

    // AUTO FUNCTIONS
    private void runBlueClose() {
        telemetry.addLine("Running BLUE NEAR Auto");
        telemetry.update();

        //sleep(2000);

        TurretPower();
        // Drive
        drive(-0.3,0,0);

        sleep(1000); // 1 second

        // Stop
        drive(0,0,0);
        sleep(6000);
        Launch();
        sleep(6000);
        drive(-0.3,0,0); //go back more
        sleep(1000);
        drive(0,0,0.2); //rotate
        sleep(700);
        drive(0,0.2,0); // align to artifact spike
        sleep(1500);
        Intake();
        drive(0.4,0,0); // pick them up
        sleep(1500);
        drive(-0.4,0,0); // go back
        sleep(1500);
        drive(0,-0.2,0); // align back to goal
        sleep(500);
        drive(0,0,-0.3); // turn to goal
        sleep(700);
        drive(0.3,0,0); // get in range of goal
        sleep(1300);
        drive(0,0,0);
        Launch();                          // fire the next 3 artifacts
        sleep(5000);






    }

    private void runBlueFar() {
        telemetry.addLine("Running BLUE FAR Auto");
        telemetry.update();

        sleep(2000);

        TurretPower(); // start turret
        // Drive
        TurretPower(); // start turret
        // Drive
        drive(0.6,0,0);
        sleep(5000); // 5 seconds

        // Stop
        drive(0,0,0);

        drive(0,0,-0.3); // facegoal
        sleep(600);
        drive(0,0,-0.3); //stop
        sleep(2000);
        Launch(); //shoot
    }

    private void runRedFar() {
        telemetry.addLine("Running RED FAR Auto");
        telemetry.update();

        sleep(2000);

        TurretPower(); // start turret
        // Drive
        drive(0.6,0,0);
        sleep(5000); // 5 seconds

        // Stop
        drive(0,0,0);

        drive(0,0,0.3); //
        sleep(600);
        drive(0,0,0.3); //stop
        sleep(2000);
        Launch(); //shoot
    }

    private void runRedClose() {
        telemetry.addLine("Running RED NEAR Auto");
        telemetry.update();
        //sleep(2000);

        TurretPower();
        // Drive
        drive(-0.3,0,0);

        sleep(1000); // 1 second

        // Stop
        drive(0,0,0);
        sleep(6000);
        Launch();
        sleep(6000);
        drive(-0.3,0,0); //go back more
        sleep(1000);
        drive(0,0,-0.2); //rotate
        sleep(700);
        drive(0,-0.2,0); // align to artifact spike
        sleep(1500);
        Intake();
        drive(0.4,0,0); // pick them up
        sleep(1500);
        drive(-0.4,0,0); // go back
        sleep(1500);
        drive(0,0.2,0); // align back to goal
        sleep(500);
        drive(0,0,0.3); // turn to goal
        sleep(700);
        drive(0.3,0,0); // get in range of goal
        sleep(1300);
        drive(0,0,0);
        Launch();                          // fire the next 3 artifacts
        sleep(5000);

    }
}