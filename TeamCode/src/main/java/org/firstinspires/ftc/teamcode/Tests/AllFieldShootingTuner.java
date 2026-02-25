package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class AllFieldShootingTuner extends OpMode {
    AFSTUNERservicehelper serviceHelper = new AFSTUNERservicehelper();
    private boolean prevDpadUp = false;
    private boolean prevDpadDown = false;

    private boolean prevDpadLeft = false;
    private boolean prevDpadRight = false;
    double[] stepSizes = {50.0,10.0, 1.0, 0.1, 0.01};
    int stepIndex = 1;

    @Override
    public void init() {
        serviceHelper.init(hardwareMap, "FALSE");



    }

    public void loop() {



        //INTAKE/OUTTAKE CODE
        if (gamepad1.left_trigger > 0.1) {
            serviceHelper.SetIntakePower(1.0);
            serviceHelper.SetServoConFrontPower(-1.0);
            serviceHelper.setFeederPower(0.7);
            serviceHelper.setIntakeServoPower(-1.0);
            //serviceHelper.SetTurretVelocity();

        } else if (gamepad1.right_trigger > 0.1) {
            serviceHelper.SetTurretVelocity();

            if (serviceHelper.isTurretAtSpeed()) {
                serviceHelper.SetIntakePower(1.0);
                serviceHelper.SetServoConFrontPower(-1.0);
                serviceHelper.setFeederPower(-1.0);
                serviceHelper.setIntakeServoPower(-1.0);
            }

        } else {
            serviceHelper.SetIntakePower(0.0);
            serviceHelper.SetServoConFrontPower(0.0);
            serviceHelper.setFeederPower(0.0);
            serviceHelper.setIntakeServoPower(0.0);
            //serviceHelper.SetTurretOFF();
        }


        //TUNING
        if (gamepad1.bWasPressed()) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        if (gamepad1.dpad_up && !prevDpadUp) {
            serviceHelper.Velocity += stepSizes[stepIndex];
        }
        if (gamepad1.dpad_down && !prevDpadDown) {
            serviceHelper.Velocity -= stepSizes[stepIndex];
        }

        if (gamepad1.dpad_right && !prevDpadRight) {
            serviceHelper.LiftHood();
        }
        if (gamepad1.dpad_left && !prevDpadLeft) {
            serviceHelper.LowerHood();
        }



        prevDpadUp = gamepad1.dpad_up;
        prevDpadDown = gamepad1.dpad_down;
        prevDpadLeft = gamepad1.dpad_left;
        prevDpadRight = gamepad1.dpad_right;


        //important values and directions
        telemetry.addData("HOOD SERVO POSITION: ","%.2f", serviceHelper.getHoodPosition());
       // telemetry.addData("\nTURRET P VALUE: ", serviceHelper.P);
        //telemetry.addData("\nTURRET F VALUE: ", serviceHelper.F);
        telemetry.addData("\nTURRET VELOCITY: ", serviceHelper.getTurretVelocity());
        telemetry.addData("\nDISTANCE FROM TAG(USE THIS): ","%.2f", serviceHelper.getDistance());
        telemetry.addData("\nSTEP SIZE: ", "%.4f", stepSizes[stepIndex]);



        telemetry.addLine("\nTUNING DIRECTIONS:");
        telemetry.addLine("\nDPAD UP/DOWN  TO INCREMENT TURRET VELOCITY(ticks per second)");
        //telemetry.addLine("\nDPAD LEFT/RIGHT  TO INCREMENT F VALUE");
        telemetry.addLine("\nCLICK B TO CHANGE STEPSIZE");
        telemetry.addLine("\nLEFT/RIGHT TRIGGER FOR INTAKE AND OUTTAKE RESPECTIVELY");
        telemetry.addLine("\nDPAD LEFT/RIGHT TO CHANGE HOOD ANGLE(or hand)");



    }
    @Override
    public void stop() {
        serviceHelper.SetIntakePower(0.0);
        serviceHelper.SetServoConFrontPower(0.0);
        serviceHelper.setServoConPower(0.0);
        serviceHelper.setFeederPower(0.0);
        serviceHelper.setIntakeServoPower(0.0);
        serviceHelper.SetTurretOFF();
        serviceHelper.stopLimelight();
    }
}



