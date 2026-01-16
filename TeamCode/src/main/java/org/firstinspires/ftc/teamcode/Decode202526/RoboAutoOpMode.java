package org.firstinspires.ftc.teamcode.Decode202526;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Disabled
@Autonomous
public class RoboAutoOpMode extends OpMode {

    private DcMotor frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;



    final double DRIVE_POWER = 0.25;

    final double WHEEL_DIAMETER_MM = 104; //mecanum wheel
    final double ENCODER_TICKS_PER_REV = 537.6;
    final double TICKS_PER_MM = (ENCODER_TICKS_PER_REV / (WHEEL_DIAMETER_MM * Math.PI));


    //create an enum to select Alliance
    private enum Alliance {
        NONE,
        RED,
        BLUE;
    }

    //create the instance for Alliance enum and default to NONE
    private Alliance alliance = Alliance.NONE;


    double forward, strafe, rotate;
    //public HardwareMap hwMap;



    @Override
    public void init() {
        //Initialize the hardware variables
        frontLeftMotor = hardwareMap.get(DcMotor.class, "front_left");
        frontRightMotor = hardwareMap.get(DcMotor.class, "front_right");
        backLeftMotor = hardwareMap.get(DcMotor.class, "back_left");
        backRightMotor = hardwareMap.get(DcMotor.class, "back_right");

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        frontLeftMotor.setZeroPowerBehavior(BRAKE);
        frontRightMotor.setZeroPowerBehavior(BRAKE);
        backLeftMotor.setZeroPowerBehavior(BRAKE);
        backRightMotor.setZeroPowerBehavior(BRAKE);

        telemetry.addData("Status", "Initialized");

    }

    //init_loop runs REPEATEDLY after the driver hits INIT, but before they hit START.
    @Override
    public void init_loop() {


        //allow the driver to select which alliance we are on using the gamepad
        if (gamepad1.b) {
            alliance = Alliance.RED;
        } else if (gamepad1.x) {
            alliance = Alliance.BLUE;
        }

        telemetry.addData("Press X", "for BLUE");
        telemetry.addData("Press B", "for RED");
        telemetry.addData("Selected Alliance", alliance);

    }

    @Override
    public void loop() {

        autoDrive(DRIVE_POWER,5,DistanceUnit.INCH);

        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    public void autoDrive(double power,double distance, DistanceUnit distanceUnit) {

        double targetPosition = (distanceUnit.toMm(distance) * TICKS_PER_MM);

        frontLeftMotor.setTargetPosition((int) targetPosition);
        frontRightMotor.setTargetPosition((int) targetPosition);
        backLeftMotor.setTargetPosition((int) targetPosition);
        backRightMotor.setTargetPosition((int) targetPosition);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeftMotor.setPower(Math.abs(power));
        frontRightMotor.setPower(Math.abs(power));
        backLeftMotor.setPower(Math.abs(power));
        backRightMotor.setPower(Math.abs(power));

    }

}
