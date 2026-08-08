package org.firstinspires.ftc.teamcode.mechanisms.OLD;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;


@TeleOp
public class AprilTagWebcamExample extends OpMode {


    AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();

    @Override
    public void init() {
        aprilTagWebcam.init(hardwareMap, telemetry);

    }

    @Override
    public void loop() {
        // update the vision portal
        AprilTagWebcam.update();
        AprilTagDetection id20 = AprilTagWebcam.getTagBySpecificId(20);
        if (id20 != null) {
            telemetry.addData("id20 String", id20.toString());
        }
    }


}
