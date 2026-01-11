package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "AutoSelection", group = "Auto")
public class AutoSelection extends LinearOpMode {

    // Define your auto modes
    enum AutoMode {
        BLUE_NEAR,
        BLUE_FAR,

        RED_NEAR,

        RED_FAR
    }

    AutoMode selectedMode = AutoMode.BLUE_NEAR;

    // Button state tracking (prevents rapid toggling)
    boolean lastLeft = false;
    boolean lastRight = false;
    boolean lastUp = false;

    boolean lastDown = false;

    boolean lastGuide = false;
    boolean confirmed = false;

    @Override
    public void runOpMode() {

        // ================= PRE-START MENU =================
        while (!isStarted() && !isStopRequested()) {

            // Navigate options
            if (gamepad1.dpad_right && !lastRight) {
                selectedMode = AutoMode.BLUE_FAR;
            }

            if (gamepad1.dpad_left && !lastLeft) {
                selectedMode = AutoMode.BLUE_NEAR;
            }
            if (gamepad1.dpad_up && !lastUp) {
                selectedMode = AutoMode.RED_FAR;
            }
            if (gamepad1.dpad_down && !lastDown) {
                selectedMode = AutoMode.RED_NEAR;
            }

            // Confirm selection with Logitech center button
            if (gamepad1.guide && !lastGuide) {
                confirmed = true;
            }

            telemetry.addLine("Select Auto");
            telemetry.addData("Selected Auto", selectedMode);
            telemetry.addData("Confirmed", confirmed);
            telemetry.addLine("D-Pad right:BLUE_FAR-C1, left:BLUE_NEAR-A5, up:RED_FAR-D1, down:RED_NEAR-E5");
            telemetry.addLine("Center (Logitech) Button: Confirm");
            telemetry.addLine("Press START when ready");
            telemetry.update();

            // Save last states
            lastLeft = gamepad1.dpad_left;
            lastRight = gamepad1.dpad_right;
            lastGuide = gamepad1.guide;

            sleep(50);
        }

        waitForStart();

        if (isStopRequested()) return;

        // ================= RUN SELECTED AUTO =================
        switch (selectedMode) {
            case BLUE_NEAR:
                runBlueClose();
                break;

            case BLUE_FAR:
                runBlueFar();
                break;

            case RED_FAR:
                runRedFar();
                break;

            case RED_NEAR:
                runRedClose();
                break;
        }
    }

    // ================= AUTO ROUTINES =================
    private void runBlueClose() {
        telemetry.addLine("Running BLUE CLOSE Auto");
        telemetry.update();

        sleep(2000);
    }

    private void runBlueFar() {
        telemetry.addLine("Running BLUE FAR Auto");
        telemetry.update();

        sleep(2000);
    }

    private void runRedFar() {
        telemetry.addLine("Running BLUE FAR Auto");
        telemetry.update();

        sleep(2000);
    }

    private void runRedClose() {
        telemetry.addLine("Running BLUE FAR Auto");
        telemetry.update();

        sleep(2000);
    }
}
