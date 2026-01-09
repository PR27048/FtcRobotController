package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

//@TeleOp
@Disabled
public class DcMotorTest extends OpMode {
    private DcMotor motor;
    public double motor1speed = 0.1;

    @Override
    public void init() {
        telemetry.addData("DcMotor", "Test");

        motor = hardwareMap.get(DcMotor.class, "motor1");
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }
    @Override
    public void loop() {
    motor.setPower(motor1speed);
    }


}