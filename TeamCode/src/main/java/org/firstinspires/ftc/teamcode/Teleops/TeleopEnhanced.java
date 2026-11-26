package org.firstinspires.ftc.teamcode.Teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

@TeleOp
public class TeleopEnhanced extends OpMode {
    double forward, strafe, rotate, speed;
    double kP = 0.026; // autotracking tune value

    private boolean initprevup = false;
    private boolean initprevdown = false;
    double Velocity;
    boolean Undertempo;

    UtilitiesEnhanced utils = new UtilitiesEnhanced();
    UtilitiesEnhanced.RelativeTagEstimate tagEstimate;

    @Override
    public void init() {
        utils.init(hardwareMap, "FALSE");

        utils.SetGate(0.07);
        utils.SetSweep(0.1);
    }

    @Override
    public void init_loop() {
        boolean upPressed = gamepad1.dpad_up && !initprevup;
        boolean downPressed = gamepad1.dpad_down && !initprevdown;

        // select alliance color menu in init
        initprevup = gamepad1.dpad_up;
        initprevdown = gamepad1.dpad_down;
        String selection;
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
            return;
        }

        if (utils.HiveUp()) {
            rotate = utils.TX() * kP;
        } else if (tagEstimate != null && tagEstimate.valid) {
            // Limelight lost sight: steer using odometry-tracked bearing to tag
            // Flip sign here if your chassis turns the opposite direction
            rotate = -tagEstimate.tagBearingDeg * kP;
        } else {
            rotate = gamepad1.right_stick_x * 2.25;
        }

        rotate = Range.clip(rotate, -0.4, 0.4);
    }

    @Override
    public void start() {
        utils.SetShooterVelocity(1450); // Turret power
    }
    @Override
    public void loop() {
        // Update Pinpoint odometry & relative tag tracking once per loop
        tagEstimate = utils.updateRelativeTagTracking();

        double x = utils.GetDistance();
        if (x <= 0.0 && tagEstimate != null && tagEstimate.valid) {
            x = tagEstimate.tagDistance;
        }

        forward = gamepad1.left_stick_y;
        strafe  = gamepad1.left_stick_x;
        rotate  = gamepad1.right_stick_x;
        speed   = 1.0; // full speed default

        if (!Double.isNaN(x) && x > 0) {
            Velocity = Range.clip(
                    (0.0255264) * x * x * x * x
                            - 3.66515 * x * x * x
                            + 196.41257 * x * x
                            - 4653.25326 * x
                            + 42541.8995,
                    1430, 1500
            );

            // Velocity += 230; // change equations
            utils.SetShooterVelocity(Velocity); // Turret power
        }

        if (gamepad1.right_bumper || gamepad2.right_bumper) {
            utils.SetSweep(0.92);
            utils.Setintake(0.7, 0.36);
        } else {
            utils.SetSweep(0.07);
        }

        if (gamepad1.left_trigger > 0.1 || gamepad2.left_trigger > 0.1) { // intake
            utils.Setintake(0.7, 0.36);
        } else if (gamepad1.left_bumper || gamepad2.right_trigger > 0.1) {
            utils.Setintake(-0.5, -0.5);
        } else if (gamepad1.right_trigger > 0.1) { // outtake / auto-align
            AlignToHive();
            utils.SetGate(0.69);

            if (utils.SlingerUp() && (utils.IsAligned() || gamepad1.left_trigger > 0.1)) {
                utils.Setintake(0.7, 0.52); // shooting speed adjust
                utils.SetFeeder(1.0);
            } else {
                utils.Setintake(0.0, 0.0);
                utils.SetFeeder(0.0);
            }
        } else {
            utils.Setintake(0.0, 0.0);
            utils.SetFeeder(0.0);
            utils.SetGate(0.035);
        }

        utils.drive(forward, strafe, rotate, speed);

        Undertempo = utils.getTurretVelocity() < Velocity;

        // Telemetry updates
        telemetry.addData("tx:", utils.TX());
        telemetry.addData("aligned?:", utils.IsAligned());
        telemetry.addData("Distance:", x);
        telemetry.addData("Tag Track Valid:", tagEstimate != null && tagEstimate.valid);
        telemetry.addData("Fresh LL Lock:", tagEstimate != null && tagEstimate.fromFreshLimelight);
        if (tagEstimate != null && tagEstimate.valid) {
            telemetry.addData("Moved Fwd / Left (in):", "%.1f / %.1f",
                    tagEstimate.movedForward, tagEstimate.movedLeft);
            telemetry.addData("Moved Heading (deg):", "%.1f", tagEstimate.movedHeadingDeg);
            telemetry.addData("Tag Rel Fwd / Left (in):", "%.1f / %.1f",
                    tagEstimate.tagRelForward, tagEstimate.tagRelLeft);
            telemetry.addData("Tag Bearing (deg):", "%.1f", tagEstimate.tagBearingDeg);
        }
        telemetry.addData("Commanded Velocity:", Velocity);
        telemetry.addData("Actual Velocity", utils.getTurretVelocity());
        telemetry.addData("Running low:", Undertempo);
        telemetry.update();
    }

    @Override
    public void stop() {
        utils.Setintake(0, 0);
    }
} // End of class