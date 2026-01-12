package org.firstinspires.ftc.teamcode.testcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
@Disabled
@TeleOp
public class IMUTesting extends OpMode {

    private IMU imu;
    double heading;

    @Override
    public void init() {
        imu = HardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot revOrientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);

        imu.initialize(new IMU.Parameters(revOrientation));
    }

    @Override
    public void loop () {

        heading = this.getHeading(AngleUnit.DEGREES);
       // telemetry.addData("Heading",this.getHeading(AngleUnit.RADIANS));
        telemetry.addData("Heading",this.getHeading(AngleUnit.DEGREES));


        if (heading < 0.5 && heading > -0.5) {
            telemetry.addLine("Heading1");
        }
        else if (heading > 0.5) {
            telemetry.addLine("Heading2");
        }
        else {
            telemetry.addLine("Heading3");
        }

    }
    public double getHeading(AngleUnit angleUnit) {
        //return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        return imu.getRobotYawPitchRollAngles().getYaw(angleUnit);

    }
}
