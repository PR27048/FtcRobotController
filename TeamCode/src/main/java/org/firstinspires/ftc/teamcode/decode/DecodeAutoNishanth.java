package org.firstinspires.ftc.teamcode.decode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.decode.ServiceHelperNishanth;

@Autonomous
public class DecodeAutoNishanth extends OpMode {

    ServiceHelperNishanth serviceHelper = new ServiceHelperNishanth();

   // double IntakePower = 0.8;
    double TurretPower = 0.55;

    @Override
    public void init() {

        serviceHelper.init(hardwareMap);
    }

    @Override
    public void start() {
     //   serviceHelper.SetIntakePower(IntakePower);
        serviceHelper.SetServoConIntakePower(-0.8);
        serviceHelper.SetTurretPower(TurretPower);
        serviceHelper.SetServoConFrontPower(-0.8);
        serviceHelper.SetServoConBackPower(-0.8);
        serviceHelper.drive(0.75, 0.0, 0.0);
        try { Thread.sleep(2000); } catch (Exception e) {}
        serviceHelper.drive(0.0, 0.0, 0.0);
        serviceHelper.SetTurretPower(TurretPower);

        serviceHelper.SetServoConBackPower(0.8);
        try { Thread.sleep(500); } catch (Exception e) {}
        serviceHelper.SetServoConFrontPower(0.0);
        serviceHelper.SetServoConBackPower(0.0);
    }

    @Override
    public void loop() {

    }

    @Override
    public void stop() {

        serviceHelper.drive(0.0, 0.0, 0.0);
    }
}
