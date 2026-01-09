package org.firstinspires.ftc.teamcode.ServoCode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class ServoConOpMode extends OpMode {
    ServoConDrive bench = new ServoConDrive();

    @Override
    public void init() {
        bench.ServoInit(hardwareMap);
    }

    @Override
    public void loop() {

        if (gamepad1.right_trigger > 0.1) {
            bench.setServoCon(1.0);
        }
        else {
            bench.setServoCon(0.0);
        }
    }
}
