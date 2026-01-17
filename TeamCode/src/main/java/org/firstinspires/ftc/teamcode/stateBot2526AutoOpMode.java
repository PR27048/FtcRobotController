package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous
public class stateBot2526AutoOpMode extends OpMode {

    stateBot2526Drive drive = new stateBot2526Drive();

    @Override
    public void init() {

        drive.init(hardwareMap);
    }

    @Override
    public void loop() {
        drive.drive(0.5, 0.0, 0.0);
        try { Thread.sleep(2000); } catch (Exception e) {}
        drive.drive(0.0, 0.0, 0.0);
    }

    @Override
    public void stop() {
        drive.drive(0.0, 0.0, 0.0);
    }
}