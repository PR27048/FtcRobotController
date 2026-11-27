package org.firstinspires.ftc.teamcode.Autos;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import java.util.List;

/*
 * Red alliance from the bottom start. Coordinates are exactly as in the visualizer files:
 *   path1 - path9:        red-autofullpath-HiveChk.pp.json
 *   tipPath6 - tipPath11: red-autofullpath-HiveChkiftip.pp
 * Heading is the intake (front) direction. The start heading is 270 to match path1
 * (the file's start says 90, which makes Pedro drive the whole field mirrored).
 *
 *  1. Throw preloaded pollen
 *  2. path1 - path3: back off, strafe to the corner, drive into the pollen (intake on)
 *  3. Wait 1.5 s so a hive that is tipping has time to finish
 *  4. path4 - path5: back off the wall, drive to the throw spot
 *  5. Camera check:
 *       hive standing: throw, path6 - path9 (flower grab), then tipPath6 - tipPath11
 *       hive tipped:   keep the pollen, tipPath6 - tipPath11
 *     tipPath9 is the other side throw spot, tipPath11 is park.
 */
@Autonomous(name = "RedAutoHiveChkUpd")
public class RedAutoHiveChkUpd extends OpMode {

    private final AutoUtilities utilities = new AutoUtilities();

    private final PoseFactory poseFactory = PoseFactory.degrees();

    // red-autofullpath-HiveChk.pp.json
    private final Pose start = poseFactory.of(57.2822, 7.8168, 270);
    private final Pose path1 = poseFactory.of(57.2801, 14.9771, 270);
    private final Pose path2 = poseFactory.of(9.6687, 15.0855, 270);
    private final Pose path3 = poseFactory.of(9.4568, 9.7650, 270);
    private final Pose path4 = poseFactory.of(9.8439, 11.4409, 270);
    private final Pose path5 = poseFactory.of(57.3992, 9.8682, 270);
    private final Pose path6 = poseFactory.of(20.8259, 46.6824, 180);
    private final Pose path7 = poseFactory.of(13.8742, 46.6210, 180);
    private final Pose path8 = poseFactory.of(21.6523, 46.6141, 180);
    private final Pose path9 = poseFactory.of(21.6820, 48.1488, 90);

    // red-autofullpath-HiveChkiftip.pp
    private final Pose tipPath6 = poseFactory.of(20.8259, 46.6824, 90);
    private final Pose tipPath7 = poseFactory.of(36.9734, 115.0413, 90);
    private final Pose tipPath8 = poseFactory.of(59.3575, 115.0035, 90);
    private final Pose tipPath9 = poseFactory.of(59.5632, 131.7641, 90);
    private final Pose tipPath10 = poseFactory.of(59.5125, 120.6393, 90);
    private final Pose tipPath11 = poseFactory.of(14.2750, 119.1410, 90);

    // Tune values
    private static final int RED_PIPELINE = 2;
    private static final double FALLBACK_VELOCITY = 1450;  // used when the Limelight can't see the hive
    private static final double CORNER_WAIT = 1.5;         // seconds in the corner, intake running, hive settles
    private static final double FLOWER_DWELL = 0.75;       // seconds at the flower with intake running
    private static final double CAMERA_CHECK_TIME = 0.5;   // seconds to look for the hive before calling it tipped
    private static final double PATH_TIMEOUT = 3.0;        // seconds max per path, then move on

    // Time guards, seconds since start
    private static final double SKIP_OTHER_SIDE_AFTER = 21.0; // hive standing: later than this after the flower, go park
    private static final double LEAVE_FOR_PARK = 26.5;        // other side throw stops at this so the robot can park

    private Follower follower;
    private final ElapsedTime timer = new ElapsedTime();
    private final ElapsedTime pathTimer = new ElapsedTime();
    private final ElapsedTime autoTimer = new ElapsedTime();

    private int pathState = 0;
    private boolean hiveTipped = false;

    private List<LynxModule> allHubs;

    private Path line(Pose from, Pose to) {
        return Paths.line(from, to).linear(from, to);
    }

    private void followPath(Pose from, Pose to) {
        follower.follow(line(from, to));
        pathTimer.reset();
    }

    // Path finished, or it took too long (blocked by a wall or pollen)
    private boolean pathDone() {
        return !follower.isBusy() || pathTimer.seconds() >= PATH_TIMEOUT;
    }

    @Override
    public void init() {

        // Utilities must init before the follower so Pedro's motor directions win
        utilities.init(hardwareMap, "auto");
        utilities.SetPipeline(RED_PIPELINE);
        utilities.SetGate(0.07);

        follower = Constants.create(hardwareMap);
        follower.setPose(start);

        // Utilities puts the drive motors in RUN_USING_ENCODER and BRAKE. Put them back the way they
        // were during Pedro tuning: velocity PID fights Pedro (shaking), and BRAKE stops much harder
        // than the coasting deceleration Pedro was tuned with (sudden stops).
        for (String name : new String[]{"front_left", "front_right", "back_left", "back_right"}) {
            DcMotor motor = hardwareMap.get(DcMotor.class, name);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

        // Faster loop for the follower, cache is cleared once at the start of every loop()
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    @Override
    public void start() {
        autoTimer.reset();
        utilities.SetShooterVelocity(FALLBACK_VELOCITY);
        utilities.BeginShot(FALLBACK_VELOCITY);
        pathState = 1;
    }

    @Override
    public void loop() {
        // Refresh all hub sensor/encoder data once at the start of the loop
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }

        follower.update();

        switch (pathState) {
            case 1: // throw preloaded pollen
                if (utilities.UpdateShot()) {
                    followPath(start, path1);
                    pathState = 2;
                }
                break;

            case 2:
                if (pathDone()) {
                    followPath(path1, path2);
                    pathState = 3;
                }
                break;

            case 3: // drive into the corner pollen with intake on
                if (pathDone()) {
                    utilities.Setintake(0.7, 0.5);
                    followPath(path2, path3);
                    pathState = 4;
                }
                break;

            case 4:
                if (pathDone()) {
                    timer.reset();
                    pathState = 5;
                }
                break;

            case 5: // wait so a tipping hive has settled, intake keeps grabbing
                if (timer.seconds() >= CORNER_WAIT) {
                    followPath(path3, path4);
                    pathState = 6;
                }
                break;

            case 6: // drive to the throw spot, shooter spins up on the way
                if (pathDone()) {
                    utilities.Setintake(0.0, 0.0);
                    utilities.SetShooterVelocity(FALLBACK_VELOCITY);
                    followPath(path4, path5);
                    pathState = 7;
                }
                break;

            case 7:
                if (pathDone()) {
                    timer.reset();
                    pathState = 8;
                }
                break;

            case 8: // camera check: hive standing -> throw here, hive tipped -> go to the other side
                if (utilities.HiveUp()) {
                    hiveTipped = false;
                    utilities.BeginShot(FALLBACK_VELOCITY);
                    pathState = 9;
                } else if (timer.seconds() >= CAMERA_CHECK_TIME) {
                    hiveTipped = true;
                    utilities.StopShooting(); // keep the pollen
                    followPath(path5, tipPath6);
                    pathState = 15;
                }
                break;

            // ---------- hive standing: throw, then grab from the flower ----------

            case 9:
                if (utilities.UpdateShot()) {
                    followPath(path5, path6);
                    pathState = 10;
                }
                break;

            case 10: // drive into the flower with intake on
                if (pathDone()) {
                    utilities.Setintake(0.7, 0.5);
                    followPath(path6, path7);
                    pathState = 11;
                }
                break;

            case 11:
                if (pathDone()) {
                    timer.reset();
                    pathState = 12;
                }
                break;

            case 12: // let the intake finish grabbing, then back out
                if (timer.seconds() >= FLOWER_DWELL) {
                    followPath(path7, path8);
                    pathState = 13;
                }
                break;

            case 13: // turn to 90
                if (pathDone()) {
                    utilities.Setintake(0.0, 0.0);
                    followPath(path8, path9);
                    pathState = 14;
                }
                break;

            case 14: // other side if there is time, otherwise straight to park
                if (pathDone()) {
                    if (autoTimer.seconds() >= SKIP_OTHER_SIDE_AFTER) {
                        followPath(path9, tipPath11);
                        pathState = 21;
                    } else {
                        followPath(path9, tipPath6);
                        pathState = 15;
                    }
                }
                break;

            // ---------- both: other side of the hive (tipPath6 - tipPath11) ----------

            case 15: // up the left side of the hive
                if (pathDone()) {
                    followPath(tipPath6, tipPath7);
                    pathState = 16;
                }
                break;

            case 16: // across the top of the hive, shooter spins up
                if (pathDone()) {
                    utilities.SetShooterVelocity(FALLBACK_VELOCITY);
                    followPath(tipPath7, tipPath8);
                    pathState = 17;
                }
                break;

            case 17: // up to the other side throw spot
                if (pathDone()) {
                    followPath(tipPath8, tipPath9);
                    pathState = 18;
                }
                break;

            case 18:
                if (pathDone()) {
                    utilities.BeginShot(FALLBACK_VELOCITY);
                    pathState = 19;
                }
                break;

            case 19: // throw, stop early if it's time to park
                if (utilities.UpdateShot() || autoTimer.seconds() >= LEAVE_FOR_PARK) {
                    utilities.StopShooting();
                    followPath(tipPath9, tipPath10);
                    pathState = 20;
                }
                break;

            case 20: // park
                if (pathDone()) {
                    followPath(tipPath10, tipPath11);
                    pathState = 21;
                }
                break;

            case 21:
                if (pathDone()) {
                    pathState = 22;
                }
                break;

            case 22: // done
                break;
        }

        telemetry.addData("Path State", pathState);
        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData("Hive tipped", hiveTipped);
        telemetry.addData("Pollen Thrown", utilities.GetShotCount());
        telemetry.addData("Auto Time", autoTimer.seconds());
        telemetry.update();
    }

    @Override
    public void stop() {
        utilities.StopShooting();
    }
}
