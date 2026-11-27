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
 * Red alliance from the bottom start.
 * Paths 1-5 and the hive-standing branch: red-autofullpath-HiveChk.pp.json
 * Hive-tipped branch (paths 6-11):         red-autofullpath-HiveChkiftip.pp
 *
 * Heading is the intake (front) direction. At 270 the intake faces the bottom wall and the
 * shooter (back) faces the hive. At 180 the intake faces the left wall. At 90 the intake faces
 * the top wall and the shooter faces the hive. Robot is 17.5 in long (intake to shooter) x 17 in wide.
 *
 *  1. Throw preloaded pollen from the start pose
 *  2. path1 - back off the wall
 *  3. path2 - strafe to the bottom-left corner
 *  4. path3 - drive into the pollen with intake running, grab pollen
 *  5. Wait 1.5 s (intake still running) so a hive that is tipping has time to finish
 *  6. path4 - back off the bottom wall a little
 *  7. path5 - drive back to the throw spot (shooter spins up)
 *  8. Camera check at the throw spot:
 *       hive visible (standing) -> throw, then
 *         path6 - turn to 180 and line up with the Red Alliance flower on the left wall
 *         path7 - drive into the flower with intake running, grab more pollen
 *       hive not visible (tipped) -> keep the pollen and go to the other side of the hive:
 *         path6 - turn to 90 on the way to the left of the hive
 *         path7 - drive up the left side of the hive
 *         path8 - cross over the top of the hive
 *         path9 - drive up to the other side throw spot and throw
 *         path10 - back off the top wall
 *         path11 - park
 *
 * Changes from the visualizer files:
 *  - start heading 270 (it said 90, but path1 already used 270) and y 8.75 so a 17.5 in robot
 *    is touching, not inside, the wall
 *  - path3 and path4 x match path2 so the robot drives straight in and out of the corner
 *  - throw spot y 11.0 (was 9.87): with the path4 back off the drive to it stays ~2.3 in off the wall
 *  - flower grab (path7, standing) y matches the line up so the drive in is straight; heading kept
 *    at 180 so the intake faces the flower
 *  - other side throw spot (59.9, 130.08) instead of (59.56, 132.5): 132.5 puts the front of the
 *    robot 0.25 in from the top wall. 130.08 is 2.7 in from the wall and 1.4 in from the top flower.
 *    path8 and path10 use the same x so path9 and path10 are straight.
 */
@Autonomous(name = "RedAutoHiveChk")
public class RedAutoHiveChk extends OpMode {

    private final AutoUtilities utilities = new AutoUtilities();

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(57.28, 8.75, 270);
    private final Pose backOffStart = poseFactory.of(57.28, 14.98, 270);
    private final Pose cornerLineUp = poseFactory.of(9.67, 15.09, 270);
    private final Pose cornerGrab = poseFactory.of(9.67, 9.76, 270);
    private final Pose cornerBackOff = poseFactory.of(9.67, 11.44, 270);
    private final Pose throwSpot = poseFactory.of(57.4, 11.0, 270);

    // Hive standing
    private final Pose flowerLineUp = poseFactory.of(20.83, 46.68, 180);
    private final Pose flowerGrab = poseFactory.of(13.87, 46.68, 180);

    // Hive tipped
    private final Pose leftOfHive = poseFactory.of(20.83, 46.68, 90);
    private final Pose leftSideTop = poseFactory.of(36.97, 115.04, 90);
    private final Pose topLane = poseFactory.of(59.9, 115.0, 90);
    private final Pose otherSideThrow = poseFactory.of(59.9, 130.08, 90);
    private final Pose parkBackOff = poseFactory.of(59.9, 120.64, 90);
    private final Pose park = poseFactory.of(14.27, 119.14, 90);

    // Tune values
    private static final int RED_PIPELINE = 2;
    private static final double FALLBACK_VELOCITY = 1450;  // used when the Limelight can't see the hive
    private static final double CORNER_WAIT = 1.5;         // seconds in the corner, intake running, hive settles
    private static final double FLOWER_DWELL = 1.0;        // seconds at the flower with intake running
    private static final double CAMERA_CHECK_TIME = 0.5;   // seconds to look for the hive before calling it tipped
    private static final double LEAVE_FOR_PARK = 25.0;     // other side throw ends by this so park finishes by ~28 s

    // Max seconds per path. If the robot can't reach the end (wall, pollen in the way)
    // isBusy() never goes false, so move on after this instead of getting stuck.
    private static final double BACK_OFF_TIMEOUT = 1.5;
    private static final double CORNER_LINE_UP_TIMEOUT = 3.0;
    private static final double CORNER_GRAB_TIMEOUT = 1.5;
    private static final double CORNER_BACK_OFF_TIMEOUT = 1.0;
    private static final double THROW_SPOT_TIMEOUT = 3.0;
    private static final double FLOWER_LINE_UP_TIMEOUT = 3.0;
    private static final double FLOWER_GRAB_TIMEOUT = 2.0;
    private static final double LEFT_OF_HIVE_TIMEOUT = 3.0;
    private static final double LEFT_SIDE_TOP_TIMEOUT = 3.0;
    private static final double TOP_LANE_TIMEOUT = 2.0;
    private static final double OTHER_SIDE_TIMEOUT = 2.0;
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
    private Path cornerGrabToBackOff;
    private Path cornerBackOffToThrowSpot;
    private Path throwSpotToFlowerLineUp;
    private Path flowerLineUpToGrab;
    private Path throwSpotToLeftOfHive;
    private Path leftOfHiveToLeftSideTop;
    private Path leftSideTopToTopLane;
    private Path topLaneToOtherSideThrow;
    private Path otherSideThrowToParkBackOff;
    private Path parkBackOffToPark;

    private int pathState = 0;
    private boolean hiveTipped = false;

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

    // Hive tag visible = hive standing. No tag after CAMERA_CHECK_TIME = hive is down (tipped)
    private boolean hiveVisible() {
        return utilities.HiveUp();
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
        cornerGrabToBackOff = line(cornerGrab, cornerBackOff);
        cornerBackOffToThrowSpot = line(cornerBackOff, throwSpot);
        throwSpotToFlowerLineUp = line(throwSpot, flowerLineUp);
        flowerLineUpToGrab = line(flowerLineUp, flowerGrab);
        throwSpotToLeftOfHive = line(throwSpot, leftOfHive);
        leftOfHiveToLeftSideTop = line(leftOfHive, leftSideTop);
        leftSideTopToTopLane = line(leftSideTop, topLane);
        topLaneToOtherSideThrow = line(topLane, otherSideThrow);
        otherSideThrowToParkBackOff = line(otherSideThrow, parkBackOff);
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

            case 5: // wait in the corner so a tipping hive has settled, intake keeps grabbing
                if (timer.seconds() >= CORNER_WAIT) {
                    followPath(cornerGrabToBackOff, CORNER_BACK_OFF_TIMEOUT);
                    pathState = 6;
                }
                break;

            case 6: // off the wall, drive back to the throw spot
                if (pathDone()) {
                    utilities.Setintake(0.0, 0.0);
                    utilities.SetShooterVelocity(FALLBACK_VELOCITY); // spin up early to save time
                    followPath(cornerBackOffToThrowSpot, THROW_SPOT_TIMEOUT);
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
                    utilities.BeginShot(FALLBACK_VELOCITY);
                    pathState = 9;
                } else if (timer.seconds() >= CAMERA_CHECK_TIME) {
                    hiveTipped = true;
                    utilities.StopShooting(); // keep the pollen, save battery on the long drive
                    followPath(throwSpotToLeftOfHive, LEFT_OF_HIVE_TIMEOUT);
                    pathState = 14;
                }
                break;

            // ---------- hive standing ----------

            case 9: // throw grabbed pollen, then head to the flower
                if (utilities.UpdateShot()) {
                    followPath(throwSpotToFlowerLineUp, FLOWER_LINE_UP_TIMEOUT);
                    pathState = 10;
                }
                break;

            case 10: // lined up with the flower, drive into it with intake on
                if (pathDone()) {
                    utilities.Setintake(0.7, 0.5);
                    followPath(flowerLineUpToGrab, FLOWER_GRAB_TIMEOUT);
                    pathState = 11;
                }
                break;

            case 11:
                if (pathDone()) {
                    timer.reset();
                    pathState = 12;
                }
                break;

            case 12: // let the intake finish grabbing, then stop holding the pollen
                if (timer.seconds() >= FLOWER_DWELL) {
                    utilities.Setintake(0.0, 0.0);
                    pathState = 13;
                }
                break;

            case 13: // done
                break;

            // ---------- hive tipped ----------

            case 14: // turned to 90 left of the hive, drive up the left side
                if (pathDone()) {
                    followPath(leftOfHiveToLeftSideTop, LEFT_SIDE_TOP_TIMEOUT);
                    pathState = 15;
                }
                break;

            case 15: // above the hive, cross over to the throw lane, shooter spins up
                if (pathDone()) {
                    utilities.SetShooterVelocity(FALLBACK_VELOCITY);
                    followPath(leftSideTopToTopLane, TOP_LANE_TIMEOUT);
                    pathState = 16;
                }
                break;

            case 16: // drive up to the other side throw spot
                if (pathDone()) {
                    followPath(topLaneToOtherSideThrow, OTHER_SIDE_TIMEOUT);
                    pathState = 17;
                }
                break;

            case 17: // only start the throw once the robot has stopped
                if (pathDone()) {
                    utilities.BeginShot(FALLBACK_VELOCITY);
                    pathState = 18;
                }
                break;

            case 18: // throw from the other side, cut short if it's time to park
                if (utilities.UpdateShot() || autoTimer.seconds() >= LEAVE_FOR_PARK) {
                    utilities.StopShooting();
                    followPath(otherSideThrowToParkBackOff, PARK_BACK_OFF_TIMEOUT);
                    pathState = 19;
                }
                break;

            case 19: // backed off the top wall, go park
                if (pathDone()) {
                    followPath(parkBackOffToPark, PARK_TIMEOUT);
                    pathState = 20;
                }
                break;

            case 20: // park
                if (pathDone()) {
                    pathState = 13;
                }
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
