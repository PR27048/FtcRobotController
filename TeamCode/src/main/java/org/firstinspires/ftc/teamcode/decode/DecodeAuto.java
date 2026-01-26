package org.firstinspires.ftc.teamcode.decode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

//import org.firstinspires.ftc.teamcode.autos.DecodeAutoAaditya;
import org.firstinspires.ftc.teamcode.decode.ServiceHelper;

@Autonomous(name = "DecodeAuto", group = "Auto")
public class DecodeAuto extends OpMode {
    ServiceHelper serviceHelper = new ServiceHelper();
    boolean lastLeft = false;
    boolean lastRight = false;
    boolean lastUp = false;
    boolean lastDown = false;
    boolean lastGuide = false;
    enum AutoMode {
        NONE,
        BLUE_NEAR,
        BLUE_FAR,
        RED_NEAR,
        RED_FAR
    }
    AutoMode selMode = AutoMode.NONE;

    @Override
    public void init() {
        serviceHelper.init(hardwareMap);
    }

    @Override
    public void init_loop() {
        if (gamepad1.dpad_right && !lastRight) {
            selMode = AutoMode.BLUE_FAR;
        }

        if (gamepad1.dpad_left && !lastLeft) {
            selMode = AutoMode.BLUE_NEAR;
        }
        if (gamepad1.dpad_up && !lastUp) {
            selMode = AutoMode.RED_FAR;
        }
        if (gamepad1.dpad_down && !lastDown) {
            selMode = AutoMode.RED_NEAR;
        }
        telemetry.addLine("Select Auto");
        //telemetry.addData("Confirmed", confirmed);
        telemetry.addLine("D-Pad to select auto \n right:BLUE_FAR-C1 \n left:BLUE_NEAR-A5 \n up:RED_FAR-D1 \n down:RED_NEAR-E5");
        telemetry.addData("\n Selected Auto", selMode);
        //  telemetry.addLine("Center (Logitech) Button: Confirm");
        telemetry.addLine("Press START when ready");
        telemetry.update();

        lastLeft = gamepad1.dpad_left;
        lastRight = gamepad1.dpad_right;
        lastUp = gamepad1.dpad_up;
        lastDown = gamepad1.dpad_down;
        lastGuide = gamepad1.guide;
    }

    @Override
    public void loop(){
        switch (selMode) {
            case BLUE_NEAR:
                runBlueClose();
                break;

            case BLUE_FAR:
               // runBlueFar();
                break;

            case RED_FAR:
//                runRedFar();
                break;

            case RED_NEAR:
                runRedClose();
                break;
        }

        selMode = AutoMode.NONE;
    }
    private void runBlueClose() {
        telemetry.addLine("Running BLUE NEAR Auto");
        telemetry.update();
        serviceHelper.SetTurretPower(0.5);
        //sleep(2000);

        serviceHelper.SetTurretPower(0.5);
        // Drive
        serviceHelper.drive(0.3,0,0);
        try { Thread.sleep(1000); } catch (Exception e) {}

        // Stop
        serviceHelper.drive(0,0,0);
        try { Thread.sleep(6000); } catch (Exception e) {}
        serviceHelper.AutoLaunch();

        try { Thread.sleep(6000); } catch (Exception e) {}
        serviceHelper.drive(0.3,0,0); //go back more

        try { Thread.sleep(1000); } catch (Exception e) {}
        serviceHelper.drive(0,0,-0.2); //rotate

        try { Thread.sleep(700); } catch (Exception e) {}
        serviceHelper.drive(0,-0.2,0); // align to artifact spike

        try { Thread.sleep(1500); } catch (Exception e) {}
        serviceHelper.AutoIntake();
        serviceHelper.drive(-0.4,0,0); // pick them up

        try { Thread.sleep(1500); } catch (Exception e) {}
        serviceHelper.drive(0.4,0,0); // go back

        try { Thread.sleep(1500); } catch (Exception e) {}
        serviceHelper.drive(0,0.2,0); // align back to goal

        try { Thread.sleep(500); } catch (Exception e) {}
        serviceHelper.drive(0,0,0.2); // turn to goal

        try { Thread.sleep(700); } catch (Exception e) {}
        serviceHelper.drive(-0.3,0,0); // get in range of goal

        try { Thread.sleep(1300); } catch (Exception e) {}
        serviceHelper.drive(0,0,0);
        serviceHelper.AutoLaunch();                         // fire the next 3 artifacts
        try { Thread.sleep(5000); } catch (Exception e) {}

        serviceHelper.drive(0,-0.1,0); // move away from launch line
        try { Thread.sleep(500); } catch (Exception e) {}

    }

    private void runRedClose() {
        telemetry.addLine("Running RED NEAR Auto");
        telemetry.update();
        serviceHelper.SetTurretPower(0.5);
        //sleep(2000);

        // Drive
        serviceHelper.drive(-0.3,0,0);
        try { Thread.sleep(1000); } catch (Exception e) {}

        // Stop
        serviceHelper.drive(0,0,0);
        try { Thread.sleep(6000); } catch (Exception e) {}
        serviceHelper.AutoLaunch();

        try { Thread.sleep(6000); } catch (Exception e) {}
        serviceHelper.drive(-0.3,0,0); //go back more

        try { Thread.sleep(1000); } catch (Exception e) {}
        serviceHelper.drive(0,0,-0.2); //rotate

        try { Thread.sleep(700); } catch (Exception e) {}
        serviceHelper.drive(0,-0.2,0); // align to artifact spike

        try { Thread.sleep(1500); } catch (Exception e) {}
        serviceHelper.AutoIntake();
        serviceHelper.drive(0.4,0,0); // pick them up

        try { Thread.sleep(1500); } catch (Exception e) {}
        serviceHelper.drive(-0.4,0,0); // go back

        try { Thread.sleep(1500); } catch (Exception e) {}
        serviceHelper.drive(0,0.2,0); // align back to goal

        try { Thread.sleep(500); } catch (Exception e) {}
        serviceHelper.drive(0,0,0.3); // turn to goal

        try { Thread.sleep(700); } catch (Exception e) {}
        serviceHelper.drive(0.3,0,0); // get in range of goal

        try { Thread.sleep(1300); } catch (Exception e) {}
        serviceHelper.drive(0,0,0);
        serviceHelper.AutoLaunch();                         // fire the next 3 artifacts

        try { Thread.sleep(5000); } catch (Exception e) {}
        serviceHelper.drive(0,0.1,0); // move away from launch line
        try { Thread.sleep(500); } catch (Exception e) {}
    }

}
