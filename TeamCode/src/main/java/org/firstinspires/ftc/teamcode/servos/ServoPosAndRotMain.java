package org.firstinspires.ftc.teamcode.servos;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp
public class ServoPosAndRotMain extends OpMode {
    ServoPosAndRot servoObj = new ServoPosAndRot();
    public HardwareMap hwMap;

    @Override
    public void init() {
        servoObj.init(hwMap);
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            servoObj.setServoPos(-1.0);
        }
        else {
            servoObj.setServoPos(1.0);
        }

        if (gamepad1.b) {
            servoObj.setServoRot(1.0);
        }
        else {
            servoObj.setServoRot(0);
        }
    }
}
