package org.firstinspires.ftc.teamcode.pedro;


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

import java.util.List;

/*
 * Red alliance full auto from the bottom start, based on red-autofullpath.pp.json.pp.
 * Heading is the intake (front) direction. At 270 the intake faces the bottom wall and the
 * shooter (back) faces the hive. At 90 the intake faces the top wall, shooter faces the hive.
 *
 *  1. Throw preloaded pollen from the start pose
 *  2. path1-3   - back off the wall, strafe to the bottom-left corner, drive into the pollen
 *  3. path4-5   - drive up the left side, turn to 90 and stop at the camera spot
 *  4. Camera check:
 *       hive seen (not tipped) -> throw from the camera spot
 *       hive not seen (tipped) -> drive to the other side (throw spot by the top wall) and throw
 *  5. Line up under the top flower, drive into it and grab pollen
 *  6. Diagonal to the throw lane, drive up to the throw spot and throw
 *  7. Back off the wall and park
 *
 * Time guards keep the whole run under 28 s: if it is too late to grab, skip straight to park,
 * and the last throw ends early if needed so the robot always has time to park.
 *
 * Robot is 17.5 in long (intake to shooter) x 17 in wide.
 *
 * Changes from the visualizer file:
 *  - start heading 270 (it said 90, but path1 already used 270) and y 8.75 so a 17.5 in robot
 *    is touching, not inside, the wall
 *  - path3 x matches path2 so the robot drives straight into the corner
 *  - path4 ends at x 36.5 (was 38.95): keeps 3.7 in from the hive instead of 1.5 in
 *  - camera spot x 57.5 (was 58.69): the 180 degree turn there stays 1 in short of the midline
 *  - both top throws use one throw spot (59.9, 130.08), 2.7 in from the top wall and
 *    1.4 in from the top flower
 *  - flower grab at the tested (47.19, 128.33): 0.6 in short of the flower tip
 *  - back off + side step pairs merged into single diagonals (checked against the flower)
 *  - park back off shortened to y 124
 */
@Autonomous(name = "RedAutoTestUpd")
public class RedAutoTestUpd extends OpMode {

    private final AutoUtilities utilities = new AutoUtilities();

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(57.28, 8.75, 270);
    private final Pose backOffStart = poseFactory.of(57.28, 14.98, 270);
    private final Pose cornerLineUp = poseFactory.of(9.67, 15.09, 270);
    private final Pose cornerGrab = poseFactory.of(9.67, 9.76, 270);
    private final Pose leftSideTop = poseFactory.of(36.5, 110.24, 270);
    private final Pose cameraSpot = poseFactory.of(57.5, 113.57, 90);
    private final Pose throwSpot = poseFactory.of(59.9, 130.08, 90);
    private final Pose flowerLineUp = poseFactory.of(47.19, 117.93, 90);
    private final Pose flowerGrab = poseFactory.of(47.19, 128.33, 90);
    private final Pose throwLane = poseFactory.of(59.9, 119.38, 90);
    private final Pose parkBackOff = poseFactory.of(59.9, 124.0, 90);
    private final Pose park = poseFactory.of(13.58, 117.29, 90);

    // Tune values
    private static final int RED_PIPELINE = 2;
    private static final double FALLBACK_VELOCITY = 1450;  // used when the Limelight can't see the hive
    private static final double CORNER_DWELL = 0.75;       // seconds in the corner with intake running
    private static final double FLOWER_DWELL = 1.0;        // seconds at the flower with intake running
    private static final double CAMERA_CHECK_TIME = 0.5;   // seconds to look for the hive before calling it tipped

    // Time guards, seconds since start
    private static final double GRAB_CUTOFF = 18.5;        // later than this, skip the flower grab and park
    private static final double LEAVE_FOR_PARK = 25.0;     // last throw ends by this so park finishes by ~28 s

    // Max seconds per path. If the robot can't reach the end (wall, pollen in the way)
    // isBusy() never goes false, so move on after this instead of getting stuck.
    private static final double BACK_OFF_TIMEOUT = 1.5;
    private static final double CORNER_LINE_UP_TIMEOUT = 3.0;
    private static final double CORNER_GRAB_TIMEOUT = 1.5;
    private static final double LEFT_SIDE_TIMEOUT = 4.0;
    private static final double CAMERA_SPOT_TIMEOUT = 2.5;
    private static final double THROW_SPOT_TIMEOUT = 2.0;
    private static final double FLOWER_LINE_UP_TIMEOUT = 2.0;
    private static final double FLOWER_GRAB_TIMEOUT = 2.0;
    private static final double THROW_LANE_TIMEOUT = 2.0;
    private static final double PARK_BACK_OFF_TIMEOUT = 1.5;
    private static final double PARK_TIMEOUT = 3.0;

    private Follower follower;
    private final ElapsedTime timer = new ElapsedTime();
    private final ElapsedTime pathTimer = new ElapsedTime();
    private final ElapsedTime autoTimer = new ElapsedTime();
    private double pathTimeout = 0;

    private Path startToBackOff;
    private Path backOffToCornerLineUp;
    private Path cornerLineUpToGrab;
    private Path cornerToLeftSideTop;
    private Path leftSideTopToCameraSpot;
    private Path cameraSpotToThrowSpot;
    private Path cameraSpotToFlowerLineUp;
    private Path cameraSpotToPark;
    private Path throwSpotToFlowerLineUp;
    private Path flowerLineUpToGrab;
    private Path flowerGrabToThrowLane;
    private Path throwLaneToThrowSpot;
    private Path throwSpotToParkBackOff;
    private Path parkBackOffToPark;

    private int pathState = 0;
    private boolean hiveTipped = false;
    private boolean atThrowSpot = false; // where the camera-check throw happened, to pick the next path

    private List<LynxModule> allHubs;

    private Path line(Pose from, Pose to) {
        return Paths.line(from, to).linear(from, to);
    }

    private void followPath(Path path, double timeout) {
        follower.follow(path);
        pathTimeout = timeout;
        pathTimer.reset();
    }

    private boolean pathDone() {
        return !follower.isBusy() || pathTimer.seconds() >= pathTimeout;
    }

    // Hive tag visible = hive standing. No tag after CAMERA_CHECK_TIME = treat the hive as tipped
    private boolean hiveVisible() {
        return utilities.HiveUp();
    }

    // After the camera-check throw: go grab from the flower, or park if there isn't time left
    private void afterFirstHiveThrow() {
        if (autoTimer.seconds() >= GRAB_CUTOFF) {
            if (atThrowSpot) {
                followPath(throwSpotToParkBackOff, PARK_BACK_OFF_TIMEOUT);
                pathState = 18;
            } else {
                followPath(cameraSpotToPark, PARK_TIMEOUT);
                pathState = 19;
            }
        } else {
            if (atThrowSpot) {
                followPath(throwSpotToFlowerLineUp, FLOWER_LINE_UP_TIMEOUT);
            } else {
                followPath(cameraSpotToFlowerLineUp, FLOWER_LINE_UP_TIMEOUT);
            }
            pathState = 12;
        }
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

        startToBackOff = line(start, backOffStart);
        backOffToCornerLineUp = line(backOffStart, cornerLineUp);
        cornerLineUpToGrab = line(cornerLineUp, cornerGrab);
        cornerToLeftSideTop = line(cornerGrab, leftSideTop);
        leftSideTopToCameraSpot = line(leftSideTop, cameraSpot);
        cameraSpotToThrowSpot = line(cameraSpot, throwSpot);
        cameraSpotToFlowerLineUp = line(cameraSpot, flowerLineUp);
        cameraSpotToPark = line(cameraSpot, park);
        throwSpotToFlowerLineUp = line(throwSpot, flowerLineUp);
        flowerLineUpToGrab = line(flowerLineUp, flowerGrab);
        flowerGrabToThrowLane = line(flowerGrab, throwLane);
        throwLaneToThrowSpot = line(throwLane, throwSpot);
        throwSpotToParkBackOff = line(throwSpot, parkBackOff);
        parkBackOffToPark = line(parkBackOff, park);
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
            case 1: // throw preloaded pollen from the start pose
                if (utilities.UpdateShot()) {
                    followPath(startToBackOff, BACK_OFF_TIMEOUT);
                    pathState = 2;
                }
                break;

            case 2: // backed off the wall, strafe to the corner
                if (pathDone()) {
                    followPath(backOffToCornerLineUp, CORNER_LINE_UP_TIMEOUT);
                    pathState = 3;
                }
                break;

            case 3: // lined up in the corner, drive into the pollen with intake on
                if (pathDone()) {
                    utilities.Setintake(0.7, 0.5);
                    followPath(cornerLineUpToGrab, CORNER_GRAB_TIMEOUT);
                    pathState = 4;
                }
                break;

            case 4:
                if (pathDone()) {
                    timer.reset();
                    pathState = 5;
                }
                break;

            case 5: // let the intake finish grabbing, keep it running while driving up the left side
                if (timer.seconds() >= CORNER_DWELL) {
                    followPath(cornerToLeftSideTop, LEFT_SIDE_TIMEOUT);
                    pathState = 6;
                }
                break;

            case 6: // turn to 90 on the way to the camera spot, shooter spins up
                if (pathDone()) {
                    utilities.Setintake(0.0, 0.0);
                    utilities.SetShooterVelocity(FALLBACK_VELOCITY);
                    followPath(leftSideTopToCameraSpot, CAMERA_SPOT_TIMEOUT);
                    pathState = 7;
                }
                break;

            case 7:
                if (pathDone()) {
                    timer.reset();
                    pathState = 8;
                }
                break;

            case 8: // camera check: throw here if the hive is standing, otherwise go to the other side
                if (hiveVisible()) {
                    hiveTipped = false;
                    atThrowSpot = false;
                    utilities.BeginShot(FALLBACK_VELOCITY);
                    pathState = 9;
                } else if (timer.seconds() >= CAMERA_CHECK_TIME) {
                    hiveTipped = true;
                    atThrowSpot = true;
                    followPath(cameraSpotToThrowSpot, THROW_SPOT_TIMEOUT);
                    pathState = 10;
                }
                break;

            case 9: // hive standing: throw from the camera spot
                if (utilities.UpdateShot()) {
                    afterFirstHiveThrow();
                }
                break;

            case 10: // hive tipped: wait until stopped at the throw spot
                if (pathDone()) {
                    utilities.BeginShot(FALLBACK_VELOCITY);
                    pathState = 11;
                }
                break;

            case 11: // hive tipped: throw from the other side
                if (utilities.UpdateShot()) {
                    afterFirstHiveThrow();
                }
                break;

            case 12: // lined up under the flower, drive into it with intake on
                if (pathDone()) {
                    utilities.Setintake(0.7, 0.5);
                    followPath(flowerLineUpToGrab, FLOWER_GRAB_TIMEOUT);
                    pathState = 13;
                }
                break;

            case 13:
                if (pathDone()) {
                    timer.reset();
                    pathState = 14;
                }
                break;

            case 14: // let the intake finish grabbing, keep it running while backing off
                if (timer.seconds() >= FLOWER_DWELL) {
                    utilities.SetShooterVelocity(FALLBACK_VELOCITY); // spin up early to save time
                    followPath(flowerGrabToThrowLane, THROW_LANE_TIMEOUT);
                    pathState = 15;
                }
                break;

            case 15: // drive up to the throw spot
                if (pathDone()) {
                    utilities.Setintake(0.0, 0.0);
                    followPath(throwLaneToThrowSpot, THROW_SPOT_TIMEOUT);
                    pathState = 16;
                }
                break;

            case 16: // only start the throw once the robot has stopped at the throw spot
                if (pathDone()) {
                    utilities.BeginShot(FALLBACK_VELOCITY);
                    pathState = 17;
                }
                break;

            case 17: // throw grabbed pollen, cut short if it's time to park
                if (utilities.UpdateShot() || autoTimer.seconds() >= LEAVE_FOR_PARK) {
                    utilities.StopShooting();
                    followPath(throwSpotToParkBackOff, PARK_BACK_OFF_TIMEOUT);
                    pathState = 18;
                }
                break;

            case 18: // backed off the wall, go park
                if (pathDone()) {
                    followPath(parkBackOffToPark, PARK_TIMEOUT);
                    pathState = 19;
                }
                break;

            case 19: // park
                if (pathDone()) {
                    utilities.StopShooting();
                    pathState = 20;
                }
                break;

            case 20:
                break;
        }

        telemetry.addData("Path State", pathState);
        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData("Hive tipped", hiveTipped);
        telemetry.addData("Commanded Velocity", utilities.GetShotVelocity());
        telemetry.addData("Shooter1 Velocity", utilities.getTurretVelocity());
        telemetry.addData("Shooter2 Velocity", utilities.getShooter2Velocity());
        telemetry.addData("Pollen Thrown", utilities.GetShotCount());
        telemetry.addData("Slinger Up", utilities.SlingerUp());
        telemetry.addData("Auto Time", autoTimer.seconds());
        telemetry.update();
    }

    @Override
    public void stop() {
        utilities.StopShooting();
    }
}
