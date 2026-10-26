package org.firstinspires.ftc.teamcode.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Teleops.Utilities;
@Disabled
@Autonomous(name = "Red Auto Timed", group = "Autonomous")
public class RedAutoTimed extends LinearOpMode {
    Utilities utils = new Utilities();

    @Override
    public void runOpMode() {
        // Initialization phase
        utils.init(hardwareMap, "FALSE");

        // Wait for the driver to press START on the Driver Station
        waitForStart();

        if (opModeIsActive()) {
            utils.SetShooterVelocity(1185);

            utils.drive(0.3, 0, 0, 1);
            sleep(300); // Built-in LinearOpMode sleep
            utils.drive(0,0,0,0);


            utils.SetGate(0.7);
            sleep(7000);

            utils.Setintake(0.5);
            sleep(3000);
            utils.SetGate(0.07);
            utils.drive(0, 0, 0.8, 0.8);
            sleep(400);
            utils.drive(0,-0.3,0,0.3);
            sleep(400);
            utils.drive(-0.4,0,0,0.4);
            sleep(1760);
            utils.drive(0,0,0,0);
            sleep(600);
            utils.Setintake(0.0);
            utils.drive(0.4,0,0,0.4);
            sleep(600);
            utils.drive(0,0,0.3,0.3);
            sleep(1020);
            utils.drive(-0.2,0,0,2);
            sleep(3000);
            utils.drive(0,0,0.5,0.5);
            sleep(200);


        }
    }
}