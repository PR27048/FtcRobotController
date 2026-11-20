package org.firstinspires.ftc.teamcode.pedro;

import org.firstinspires.ftc.teamcode.Teleops.Utilities;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/*
 * Red alliance full auto, following requirement/red-autofull (Pedro visualizer).
 *
 *  1. Throw preloaded pollen into the hive from the start pose
 *  2. path1 - turn to 180
 *  3. path2 - drive to the corner with intake running, collect pollen
 *  4. path3 - drive up the left side to the top of the field (shooter spins up)
 *  5. path4 - check the hive QR code / AprilTag with the Limelight
 *             seen     -> throw here, then path5
 *             not seen -> path5 to the other side of the hive and throw there
 *  6. path6 - park
 */
@Disabled
@Autonomous(name = "RedAutoTest")
public class RedAutoTest extends OpMode {

    private final Utilities utils = new Utilities();

    private final PoseFactory poseFactory = PoseFactory.degrees();

    // Poses copied from requirement/red-autofull.java
    // Heading is the intake (front) direction. Robot starts with the shooter (back) facing the hive,
    // so the intake faces the wall at 270. Visualizer had 90 which flips the whole field.
    private final Pose start = poseFactory.of(57.2822, 7.8168, 270);
    private final Pose point1 = poseFactory.of(57.2801, 14.9771, 270);
    private final Pose point2Start = poseFactory.of(57.2801, 14.9771, 270);
    private final Pose point2 = poseFactory.of(11.5111, 14.5328, 270);
    private final Pose point3Start = poseFactory.of(11.5111, 14.5328, 270);
    private final Pose point3 = poseFactory.of(9.4568, 9.765, 270);
    private final Pose point4Start = poseFactory.of(9.4568, 9.765, 270);
    private final Pose point4 = poseFactory.of(34.6385, 105.0258, 270);
    private final Pose point5Start = poseFactory.of(34.6385, 105.0258, 135);
    private final Pose point5 = poseFactory.of(57.02128, 121.177, 90);
    private final Pose point6Start = poseFactory.of(57.02128, 121.177, 90);
    private final Pose point6 = poseFactory.of(14.0232, 114.2078, 100);
    // Tune values
    private static final int RED_PIPELINE = 2;
    private static final double FALLBACK_VELOCITY = 1450;// 1185;  // used when the Limelight can't see the hive (RedAutoTimed value)
    private static final double SPINUP_TIMEOUT = 1.5;      // seconds to wait for shooter to reach speed
    private static final double FEED_TIME = 2.5;           // seconds to feed pollen into the shooter
    private static final double INTAKE_DWELL = 1.0;        // seconds to sit at the corner with intake running
    private static final double QR_SCAN_TIME = 1.0;        // seconds to look for the hive QR code

    private Follower follower;
    private final ElapsedTime timer = new ElapsedTime();
    private final ElapsedTime shotTimer = new ElapsedTime();

    private Path firstPath;
    private Path secondPath;
    private Path thirdPath;
    private Path fourthPath;
    private Path fifthPath;
    private Path sixthPath;

    private int pathState = 0;
    private double velocity = FALLBACK_VELOCITY;
    private boolean feeding = false;
    private boolean hiveSeen = false;

    public Path path1() {
        return Paths.line(start, point1).linear(start, point1);
    }

    public Path path2() {
        return Paths.line(point2Start, point2).linear(point2Start, point2);
    }

    public Path path3() {
        return Paths.line(point3Start, point3).linear(point3Start, point3);
    }

    public Path path4() {
        return Paths.line(point4Start, point4).linear(point4Start, point4);
    }

    public Path path5() {
        return Paths.line(point5Start, point5).linear(point5Start, point5);
    }

    public Path path6() {
        return Paths.line(point6Start, point6).linear(point6Start, point6);
    }

    // Same distance -> velocity curve as Teleop
    private double velocityFromDistance(double x) {
        return Range.clip(
                (0.0255264) * x * x * x * x
                        - 3.66515 * x * x * x
                        + 196.41257 * x * x
                        - 4653.25326 * x
                        + 42541.8995,
                1430, 1500
        );
    }


    private void updateShooterVelocity() {
        // HiveUp() null-checks the Limelight result, GetDistance() does not
        if (utils.HiveUp()) {
            double x = utils.GetDistance();
            if (!Double.isNaN(x) && x > 0) {
                velocity = velocityFromDistance(x);
            }
        }
        utils.SetShooterVelocity(velocity);
    }

    private void beginShot() {
        velocity = FALLBACK_VELOCITY;
        feeding = false;
        shotTimer.reset();
    }

    // Call every loop while shooting, returns true once all pollen has been thrown
    private boolean updateShot() {
        updateShooterVelocity();
        utils.SetGate(0.69);

        if (!feeding) {
            if (utils.isTurretAtSpeed(velocity) || shotTimer.seconds() >= SPINUP_TIMEOUT) {
                feeding = true;
                shotTimer.reset();
            }
            return false;
        }

        if (utils.SlingerUp() && utils.isTurretAtSpeed(velocity)) {
            utils.Setintake(0.7, 0.7); //shooting speeed adjust
            utils.SetFeeder(1.0);
        } else {
            utils.Setintake(0.0, 0.0);
        }

        if (shotTimer.seconds() >= FEED_TIME) {
            stopShooting();
            return true;
        }
        return false;
    }

    private void stopShooting() {
        utils.Setintake(0.0, 0.0);
        utils.SetFeeder(0.0);
        utils.SetGate(0.035);
        utils.SetShooterVelocity(0);
    }

    @Override
    public void init() {

        // Utilities must init before the follower so Pedro's motor directions win
        utils.init(hardwareMap, "auto");
        utils.SetPipeline(RED_PIPELINE);
        utils.SetGate(0.07);

        follower = Constants.create(hardwareMap);
        follower.setPose(start);

        firstPath = path1();
        secondPath = path2();
        thirdPath = path3();
        fourthPath = path4();
        fifthPath = path5();
        sixthPath = path6();
    }

    @Override
    public void start() {
        utils.SetShooterVelocity(FALLBACK_VELOCITY); // spin up while backing off the wall
        timer.reset();
        beginShot();
        pathState = 1;
    }

    @Override
    public void loop() {
        follower.update();

        switch (pathState) {
            case 1: // throw preloaded pollen from the start pose
                if (updateShot()) {
                    follower.follow(firstPath);
                    pathState = 2;
                }
                break;

            case 2: // turn done, drive to the corner with intake on
                if (!follower.isBusy()) {
                    utils.Setintake(0.7, 0.7); //uncomment after path tested
                    follower.follow(secondPath);
                    pathState = 3;
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    timer.reset();
                    pathState = 4;
                }
                break;

            case 4: // let the intake finish grabbing pollen, then drive up the field
                if (timer.seconds() >= INTAKE_DWELL) {
                    utils.Setintake(0.0, 0.0); //uncomment after path tested
                   utils.SetShooterVelocity(FALLBACK_VELOCITY); // spin up early to save time //uncomment after path tested
                    follower.follow(thirdPath);
                    pathState = 5;
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    follower.follow(fourthPath);
                    pathState = 6;
                }
                break;

            case 6:
                if (!follower.isBusy()) {
                    timer.reset();
                    pathState = 7;
                }
                break;

            case 7: // check the hive QR code: throw here if seen, otherwise go to the other side of the hive
                /*if (utils.HiveUp()) {
                    hiveSeen = true;
                    beginShot();
                    pathState = 12;
                } else if (timer.seconds() >= QR_SCAN_TIME) {
                    follower.follow(fifthPath);
                    pathState = 8;
                 }*/ //uncomment after path tested
                follower.follow(fifthPath); //delete
                pathState = 8; //delete
                break;

            case 8:
                if (!follower.isBusy()) {
                   // beginShot(); //uncomment
                    pathState = 9; //9 uncomment and replace
                }
               // pathState = 10;
                break;

            case 9: // throw collected pollen
               // if (updateShot()) { uncomment
                    follower.follow(sixthPath);
                    pathState = 10;
                //}
                break;

            case 10: // park
                if (!follower.isBusy()) {
                    stopShooting();
                    pathState = 11;
                }
                break;

            case 11:
                break;

            case 12: // hive seen at point4, throw collected pollen here
                //if (updateShot()) { uncomment
                    follower.follow(fifthPath);
                    pathState = 13;
                //}
                break;

            case 13: // already thrown, follow the visualizer path to park
                if (!follower.isBusy()) {
                    follower.follow(sixthPath);
                    pathState = 10;
                }
                break;
        }

        telemetry.addData("Path State", pathState);
        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData("Hive QR seen", hiveSeen);
        telemetry.addData("Commanded Velocity", velocity);
        telemetry.addData("Actual Velocity", utils.getTurretVelocity());
        telemetry.update();
    }

    @Override
    public void stop() {
        stopShooting();
    }
}
