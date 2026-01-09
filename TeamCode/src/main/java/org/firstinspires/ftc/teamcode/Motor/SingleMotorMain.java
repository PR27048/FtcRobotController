package org.firstinspires.ftc.teamcode.Motor;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@Disabled
@TeleOp
public class SingleMotorMain extends OpMode {
    private DcMotor motor;

    private double motorPower = 0.25;
    private double motorZeroPower = 0.0;

    @Override
    public void init (){
        motor = hardwareMap.get(DcMotor.class, "single_motor");
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(BRAKE);
    }

    @Override
    public void loop() {
        motor.setPower(motorPower);

        telemetry.log().clear();
        telemetry.addData("Current Power", motor.getPower());
        telemetry.update();

    }

    @Override
    public void stop() {
        motor.setPower(motorZeroPower);
    }
    /*
     * motor.setZeroPowerBehavior(BRAKE);
     * Setting zeroPowerBehavior to BRAKE enables a "brake mode". This causes the motor to
     * slow down much faster when it is coasting. This creates a much more controllable
     * drivetrain. As the robot stops much quicker.
     */

}
