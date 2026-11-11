package org.firstinspires.ftc.teamcode.Teleops;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

@TeleOp
public class SoftwareBreakTest extends OpMode {
    double forward, strafe, rotate, speed;
    double kP = 0.026;  //autotracking tune value

    private boolean initprevup = false;
    double Velocity;

    private boolean initprevdown = false;
    boolean Undertempo;

    Utilities utils = new Utilities();

    @Override
    public void init() {
        utils.init(hardwareMap, "FALSE");

        utils.SetGate(0.07);
        // utils.SetSweep(0.0);


    }

    public void init_loop() {
        boolean upPressed = gamepad1.dpad_up && !initprevup;
        boolean downPressed = gamepad1.dpad_down && !initprevdown;

        // select alliance color menu in init
        initprevup = gamepad1.dpad_up;
        initprevdown = gamepad1.dpad_down;
        String selection = "";
        telemetry.addLine("Select Alliance Color:");
        telemetry.addLine("\n---> DPAD_UP = RED");
        telemetry.addLine("---> DPAD_DOWN = BLUE");
        if (utils.getCurrentPipeline() == 2) {
            selection = "RED";
        } else {
            selection = "BLUE";
        }

        telemetry.addData("\nCURRENT SELECTION:", selection);
        telemetry.update();
        utils.updatePipelineMenu(upPressed, downPressed);
    }
    public void AlignToHive() {
        if (utils.IsAligned()) {
            rotate = 0.0;
        } else {
            if (utils.TX() == 0) {
                rotate = gamepad1.right_stick_x * 2.25;
            } else {
                rotate = utils.TX() * kP;
            }

            rotate = Math.max(-0.4, Math.min(0.4, rotate));
        }
    }

    @Override
    public void loop() {
        double x = utils.GetDistance();
        // double yr = utils.yawerror();
        forward = gamepad1.left_stick_y;
        strafe  = gamepad1.left_stick_x;
        rotate  = gamepad1.right_stick_x;
        speed   = 1.0; // full speed default

        utils.drive(forward, strafe, rotate, speed);














        //Telemetry updates
        telemetry.addData("tx:", utils.TX());
        telemetry.addData("aligned?:", utils.IsAligned());
        telemetry.addData("Distance:", x);
        telemetry.update();
    }

    @Override
    public void stop() {
        utils.Setintake(0,0);
    }
} //End of class