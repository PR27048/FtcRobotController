package org.firstinspires.ftc.teamcode.Motor;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

/*
P - raw power you are sending
	-more far more raw power
	-close your are less raw power

 */

/*
happy with F = 14.09 to get 1500 target velocity(rpm)
 */
public class MotorPFTuning extends OpMode {

    private DcMotorEx motor;
    private double highVelocity = 1500;
    private double lowVelocity = 900;

    private double currentTargetVelocity = highVelocity;

    double F = 0.0;
    double P = 0.0;
    double[] stepSize = {10.0,1.0,0.1,0.001,0.0001};

    int stepIndex = 1;




    @Override
    public void init () {
        motor = hardwareMap.get(DcMotorEx.class, "single_motor");
        motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motor.setDirection(DcMotorEx.Direction.REVERSE);
        //motor.setDirection(DcMotorEx.Direction.FORWARD);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P,0,0,F);
        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidfCoefficients);
        telemetry.addLine("Init Complete");
    }

    @Override
    public void loop() {
        //get gamepad commands
        //set target velocity
        //update telemetry

        //setting High or low velocity
        if(gamepad1.yWasPressed()) {
            if(currentTargetVelocity == highVelocity) {
                currentTargetVelocity = lowVelocity;
            }
            else {
                currentTargetVelocity = highVelocity;
            }
        }

        //Tuning F values
        if(gamepad1.dpadLeftWasPressed()) {
            F -= stepSize[stepIndex];
        }
        if(gamepad1.dpadRightWasPressed()) {
            F += stepSize[stepIndex];
        }

        //Tune P values
        if(gamepad1.dpadUpWasPressed()) {
            P += stepSize[stepIndex];
        }
        if(gamepad1.dpadDownWasPressed()) {
            P -= stepSize[stepIndex];
        }

        //set new PIDFCoefficients
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P,0,0,F);
        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidfCoefficients);

        //set velocity
        motor.setVelocity(currentTargetVelocity);

        double curVelocity = motor.getVelocity();
        double error = currentTargetVelocity - curVelocity;

        telemetry.addData("Target Velocity",currentTargetVelocity);
        telemetry.addData("Current Velocity","%.2f",curVelocity);
        telemetry.addData("Error","%.2f",error);
        telemetry.addLine("-----------------------------------------------");
        telemetry.addData("Tuning P","%.4f (D-Pad U/D)",P);
        telemetry.addData("Tuning F","%.4f (D-Pad L/R)",F);
        telemetry.addData("Step Size","%.4f",stepSize[stepIndex]);
    }

}
