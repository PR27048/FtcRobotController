package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.decode.ServiceHelperNishanth;

@Autonomous
public class DecodeAutoNishanth extends OpMode {

    ServiceHelperNishanth serviceHelper = new ServiceHelperNishanth();

    double IntakePower = 0.8;
    double TurretPower = 0.7;
    @Override
    public void init() {
        serviceHelper.init(hardwareMap);
    }

    @Override
    public void loop() {
        serviceHelper.drive(0.5, 0.0, 0.0);
        try { Thread.sleep(2000); } catch (Exception e) {}
        serviceHelper.drive(0.0, 0.0, 0.0);
    }

    @Override
    public void stop() {
        serviceHelper.drive(0.0, 0.0, 0.0);
    }
}
