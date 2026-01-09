package org.firstinspires.ftc.teamcode.ServoCode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class ServoPosOpMode extends OpMode {
    ServoPosDrive bench = new ServoPosDrive();
    @Override
    public void init() {
        bench.ServoInit(hardwareMap);
    }

    public void loop() {
        if (gamepad1.a) {
            bench.setServoPos(0.0);
        }
        else {
            bench.setServoPos(1.0);
        }
    }
}