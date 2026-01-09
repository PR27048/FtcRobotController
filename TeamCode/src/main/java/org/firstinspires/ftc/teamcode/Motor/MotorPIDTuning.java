package org.firstinspires.ftc.teamcode.Motor;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

//first get maxVelocity of the motor from ToGetMaxVelocityOfMotor
//https://docs.google.com/document/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/edit?tab=t.0#heading=h.61g9ixenznbx
//https://www.youtube.com/watch?v=6OH-wOsVVjg
@Disabled
@TeleOp
public class MotorPIDTuning extends OpMode {

    private DcMotorEx motor;
    private double motorZeroPower = 0.0;
    private double currentVelocity = 0.0;
    private double maxVelocity = 0.0;

    private double targetVelocity = 800.00; // 80% of the max velocity got from test ToGetMaxVelocityOfMotor
    private double maxVelocityFromTest = 2800.00; //Max velocity from Test ToGetMaxVelocityOfMotor
    private double F = 32767.00; // may change based on motor
    private double P = F * 0.1;
    //P terms oscillation, if overshot (i.e if maxVelocity > 800.00 then decrease P value)
    //if undershoot the increase P value
    private double I = P * 0.1;
    //I term nudges you to your target over time
    private double D = P * 0.01;
    //D trem applies a breaking force to control over shoot. Keep this term small to avoid noise
    //Error, noise, should be low
    private double position = 5.0;

    @Override
    public void init (){
        motor = hardwareMap.get(DcMotorEx.class, "single_motor");
        motor.setDirection(DcMotorEx.Direction.FORWARD);
        motor.setPower(motorZeroPower);
        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        motor.setVelocityPIDFCoefficients(P,I,D,F);
        motor.setPositionPIDFCoefficients(position);
        motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {

        motorVelocity(targetVelocity);

        telemetry.log().clear();
        telemetry.addData("Current Power", motor.getPower());
        telemetry.addData("Target Velocity", targetVelocity);
        telemetry.addData("Maximum Velocity", maxVelocity);
        telemetry.addData("Current Velocity", currentVelocity);
        telemetry.addData("F", F);
        telemetry.addData("P", P);
        telemetry.addData("I", I);
        telemetry.addData("D", D);
        telemetry.addData("position", position);

        telemetry.update();
    }

    public void motorVelocity(double velocity) {
        motor.setVelocity(velocity);
        currentVelocity =  motor.getVelocity();
        if(currentVelocity > maxVelocity){
            maxVelocity = currentVelocity;
        }
    }


}
