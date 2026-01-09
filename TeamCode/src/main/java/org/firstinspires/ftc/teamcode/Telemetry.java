package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@TeleOp
public class Telemetry extends OpMode {

    @Override
    public void init() {
        telemetry.addData("Hello", "Nishanth K");
    }
    @Override
    public void loop() {

    }


}