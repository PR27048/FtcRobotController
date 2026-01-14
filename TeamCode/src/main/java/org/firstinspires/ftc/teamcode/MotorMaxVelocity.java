package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

//Using Extended Dc Motor class DcMotorEx
//No PIDF, no encoder
//Getting max velocity of motor with max power(raw power)
@TeleOp
public class MotorMaxVelocity extends OpMode {

    private DcMotorEx motor;
    private double motorZeroPower = 0.0;
    private double motorMaxPower = 0.9375;
    private double currentVelocity = 0.0;
    private double maxVelocity = 0.0;

    @Override
    public void init (){
        motor = hardwareMap.get(DcMotorEx.class, "motor");
        motor.setDirection(DcMotorEx.Direction.FORWARD);
        motor.setPower(motorZeroPower);
        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(BRAKE);
    }

    @Override
    public void loop() {

        motorMaxVelocityTest();

        telemetry.log().clear();
        telemetry.addData("Current Power", motor.getPower());
        telemetry.addData("Maximum Velocity", maxVelocity);
        telemetry.addData("Current Velocity", currentVelocity);
        telemetry.update();
    }

    @Override
    public void stop() {
        motor.setPower(motorZeroPower);
    }

    public void motorMaxVelocityTest(){
        motor.setPower(motorMaxPower);
        currentVelocity = motor.getVelocity();
        if(currentVelocity > maxVelocity)
            maxVelocity = currentVelocity;
    }
}