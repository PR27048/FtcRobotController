package org.firstinspires.ftc.teamcode.NewTeleops;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@TeleOp
@Disabled
public class Teleop extends OpMode {
    double forward, strafe, rotate, speed;
    double kP = 0.026;  // autotracking tune value

    private boolean initprevup = false;
    private boolean initprevdown = false;
    boolean Undertempo;

    // Declare the Pedro Pathing Follower
    private Follower follower;
    Utilities utils = new Utilities();

    @Override
    public void init() {
        // Initialize the Follower and start teleop drive mode
        follower = Constants.create(hardwareMap);
        utils.init(hardwareMap, "FALSE");

        // utils.SetGate(0.07);
        // utils.SetSweep(0.1);
    }

    @Override
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

    @Override
    public void loop() {
        // 1. MUST update follower and grab pose every loop iteration
        follower.update();
        Pose currentPose = follower.pose();

        // 2. Drivetrain joystick inputs (Inverted Y so pushing stick forward drives forward)
        forward = -gamepad1.left_stick_y;
        strafe  =  gamepad1.left_stick_x;
        rotate  =  gamepad1.right_stick_x;
        speed   =  1.0; // full speed default

        // 3. Automated Turret (Handles auto-anchoring, odometry math, and 1:4 goBILDA servo scaling)
        utils.updateTurret(currentPose);

        // 4. Drive the robot
        utils.drive(forward, strafe, rotate, speed);

        // 5. Telemetry updates
        telemetry.addData("Robot X", currentPose.x());
        telemetry.addData("Robot Y", currentPose.y());
        telemetry.addData("Heading (Deg)", Math.toDegrees(currentPose.heading()));
        telemetry.addData("Distance:", utils.getDistance());
        telemetry.addData("Actual Velocity", utils.getTurretVelocity());
        telemetry.update();
    }

    @Override
    public void stop() {
        utils.Setintake(0, 0);
    }
} // End of class