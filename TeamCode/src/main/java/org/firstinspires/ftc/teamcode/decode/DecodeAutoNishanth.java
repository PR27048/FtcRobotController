package org.firstinspires.ftc.teamcode.decode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.decode.ServiceHelperNishanth;

@Autonomous
public class DecodeAutoNishanth extends OpMode {

    ServiceHelperNishanth serviceHelper = new ServiceHelperNishanth();

    double IntakePower = 1.0;
    double TurretPower = 0.5;

    @Override
    public void init() {

        serviceHelper.init(hardwareMap);
    }

    @Override
    public void start() {
    /*    serviceHelper.SetIntakePower(IntakePower);
        serviceHelper.SetServoConIntakePower(-0.8);
        serviceHelper.SetTurretPower(TurretPower);
        serviceHelper.SetServoConFrontPower(-0.8);
        serviceHelper.SetBackFeederPower(0.7);
        serviceHelper.drive(-0.6, 0.0, 0.0);
        try { Thread.sleep(2000); } catch (Exception e) {}
        serviceHelper.drive(0.0, 0.0, 0.0);
        serviceHelper.SetTurretPower(TurretPower);
        try { Thread.sleep(6000); } catch (Exception e) {}
        serviceHelper.SetBackFeederPower(-0.7);

        try { Thread.sleep(5000); } catch (Exception e) {}
        serviceHelper.SetServoConFrontPower(0.0);
        serviceHelper.SetBackFeederPower(0.0);*/

        serviceHelper.SetTurretPower(TurretPower);
        serviceHelper.SetServoConFrontPower(-0.8);
        serviceHelper.SetBackFeederPower(0.8);
        serviceHelper.SetServoConIntakePower(-1.0);
        serviceHelper.SetIntakePower(IntakePower);
        serviceHelper.drive(-0.05, 0.0, 0.0);
        try { Thread.sleep(1000); } catch (Exception e) {}
        serviceHelper.drive(0.0, 0.0, 0.0);
        try { Thread.sleep(6000); } catch (Exception e) {}
        serviceHelper.SetBackFeederPower(-0.8);
        try { Thread.sleep(500); } catch (Exception e) {}
        serviceHelper.SetBackFeederPower(0.8);
        serviceHelper.SetTurretPower(0.57);
        try { Thread.sleep(2000); } catch (Exception e) {}
        serviceHelper.SetBackFeederPower(-0.8);
        try { Thread.sleep(500); } catch (Exception e) {}
        serviceHelper.SetBackFeederPower(0.8);
        serviceHelper.SetTurretPower(0.555);
        try { Thread.sleep(2000); } catch (Exception e) {}
        serviceHelper.SetBackFeederPower(-0.8);
        try { Thread.sleep(5000); } catch (Exception e) {}
        serviceHelper.SetBackFeederPower(0.8);
        serviceHelper.drive(-0.5, 0.0, 0.0);
        try { Thread.sleep(500); } catch (Exception e) {}
        serviceHelper.drive(0.0, 0.0, 0.0);

    }

    @Override
    public void loop() {

    }

    @Override
    public void stop() {

        serviceHelper.drive(0.0, 0.0, 0.0);
    }
}
