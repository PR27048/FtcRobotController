package org.firstinspires.ftc.teamcode.TuningPrograms;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Teleops.Utilities;

@TeleOp

public class AFStuner extends OpMode {
    double forward, strafe, rotate, speed;
    double kP = 0.026;  //autotracking tune value
    private boolean prevDpadUp = false;
    private boolean prevDpadDown = false;

    private boolean prevDpadLeft = false;
    private boolean prevDpadRight = false;
    double Velocity = 0;

    double TransferSpeed = 0.36;


    double[] stepSizes = {50.0,10.0, 1.0, 0.1, 0.01};
    int stepIndex = 1;

    private boolean initprevup = false;
    private boolean initprevdown = false;



    private double lastDisplayedDistance = 0.0;
    private final double DEADBAND_THRESHOLD = 0.4;

    Utilities utils = new Utilities();

    @Override
    public void init() {
        utils.init(hardwareMap, "FALSE");

        utils.SetGate(0.035);
        utils.SetSweep(0.0);

    }

    public void init_loop() {
        boolean upPressed = gamepad1.dpad_up && !initprevup;
        boolean downPressed = gamepad1.dpad_down && !initprevdown;

        // select alliance color menu in init
        initprevup = gamepad1.dpad_up;
        initprevdown = gamepad1.dpad_down;
        String selection = "";
        telemetry.addLine("Select Alliance Color:");
        telemetry.addLine("\n---> DPAD_UP = RED");
        telemetry.addLine("---> DPAD_DOWN = BLUE");
        if (utils.getCurrentPipeline() == 2) {
            selection = "RED";
        } else {
            selection = "BLUE";
        }

        telemetry.addData("\nCURRENT SELECTION:", selection);
        telemetry.update();
        utils.updatePipelineMenu(upPressed, downPressed);
    }

    @Override
    public void loop() {



        forward = gamepad1.left_stick_y;
        strafe  = gamepad1.left_stick_x;
        rotate  = gamepad1.right_stick_x;
        speed   = 1.0; // full speed default
        utils.SetShooterVelocity(Velocity); //Turret ON

        if(gamepad1.left_bumper) {
            utils.SetSweep(0.7);
        } else {utils.SetSweep(0.0);}

        if (gamepad1.left_trigger > 0.1) { // intake
            utils.Setintake(0.7,0.36);
           // utils.SetFeeder(-.4);
            //utils.SetGate(0.04);

        } else if(gamepad1.right_bumper) {
            utils.Setintake(-0.7,-0.7);
        } else if (gamepad1.right_trigger > 0.1) { // outtake / auto-align

            //if(utils.isTurretAtSpeed(Velocity)) {

            utils.SetGate(0.69);
            if(utils.SlingerUp()) {
                utils.Setintake(0.7,TransferSpeed);   //  Push elements up

                utils.SetFeeder(1.0);

                if (utils.IsAligned()) {
                    rotate = 0.0; // Stay still if aligned


                } else {
                    utils.Setintake(0.0,0.0);
                }



            }

            if(utils.TX() == 0) //Completely off
            {
                rotate = gamepad1.right_stick_x * 2.25; //60 * kP

            } else {
                rotate = utils.TX() * kP;  //In sight
            }
            rotate = Math.max(-0.4, Math.min(0.4, rotate)); //turn

        } else {
            utils.Setintake(0.0,0.0);
            utils.SetFeeder(0.0);
            utils.SetGate(0.035);
        }

        utils.drive(forward, strafe, rotate, speed);

        if (gamepad1.bWasPressed()) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        if (gamepad1.dpad_up && !prevDpadUp) {
            Velocity +=stepSizes[stepIndex];
            utils.SetShooterVelocity(Velocity);
        }
        if (gamepad1.dpad_down && !prevDpadDown) {
            Velocity -=stepSizes[stepIndex];
            utils.SetShooterVelocity(Velocity);

        }
        if (gamepad1.dpad_right && !prevDpadRight) {
            TransferSpeed +=stepSizes[stepIndex];
        }
        if (gamepad1.dpad_left && !prevDpadLeft) {
            TransferSpeed -=stepSizes[stepIndex];
        }


        prevDpadUp = gamepad1.dpad_up;
        prevDpadDown = gamepad1.dpad_down;
        prevDpadLeft = gamepad1.dpad_left;
        prevDpadRight = gamepad1.dpad_right;


        //Telemetry updates

        //telemetry.addData("tx:", utils.TX());
        //telemetry.addData("aligned?:", utils.IsAligned());
        double currentDistance = utils.GetDistance();

        if (Math.abs(currentDistance - lastDisplayedDistance) >= DEADBAND_THRESHOLD) {
            lastDisplayedDistance = currentDistance;
        }

        telemetry.addData("Distance(X):", "%.2f", lastDisplayedDistance);        telemetry.addData("Velocity(Y):", Velocity);
        telemetry.addData("Actual Velocity(Y):", utils.getTurretVelocity());
        telemetry.addData("Transfer Speed: ", TransferSpeed);
        telemetry.addData("Stepsize:", stepSizes[stepIndex]);
        telemetry.update();
    }

    @Override
    public void stop() {
        utils.Setintake(0,0);
    }
} //End of class