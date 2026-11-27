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

import org.firstinspires.ftc.teamcode.Autos.AutoUtilities;

import java.util.List;

/*
 * Blue alliance, start position at the bottom wall (2nd hive side), two throws.
 * Poses from Blue2ndPos2Throw-upd.pp: the red path turned 180 degrees around the field center.
 * Heading is the intake (front) direction. At 270 the intake faces the bottom wall
 * and the shooter (back) faces the hive.
 *
 *  1. Throw preloaded pollen into the hive from the start pose
 *  2. path1 - back off the wall
 *  3. path2 - line up over the pollen at the bottom wall
 *  4. path3 - drive up to the wall with intake running, grab pollen
 *  5. path4 - back off the wall
 *  6. path5 - move over to the throw lane (shooter spins up)
 *  7. path6 - drive up near the wall to the throw spot
 *  8. Throw grabbed pollen
 *  9. path7 - back off the wall
 * 10. path8 - park
 */
@Autonomous(name = "Blue2ndPos2Throw")
public class Blue2ndPos2Throw extends OpMode {

    private final AutoUtilities utilities = new AutoUtilities();

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(81.9994, 7.845, 270);
    private final Pose point1 = poseFactory.of(81.6309, 17.2467, 270);
    private final Pose point2Start = poseFactory.of(81.6309, 17.2467, 270);
    private final Pose point2 = poseFactory.of(93.5688, 27.4451, 270);
    private final Pose point3Start = poseFactory.of(93.5688, 27.4451, 270);
    private final Pose point3 = poseFactory.of(94.3079, 13.171, 270);
    private final Pose point4Start = poseFactory.of(94.3079, 13.171, 270);
    private final Pose point4 = poseFactory.of(94.0347, 21.5781, 270);
    private final Pose point5Start = poseFactory.of(94.0347, 21.5781, 270);
    private final Pose point5 = poseFactory.of(82.0858, 15.7838, 270);
    private final Pose point6Start = poseFactory.of(82.0858, 15.7838, 270);
    private final Pose point6 = poseFactory.of(82.0759, 9.532, 270);
    private final Pose point7Start = poseFactory.of(82.0759, 9.532, 270);
    private final Pose point7 = poseFactory.of(82.4564, 15.8947, 270);
    private final Pose point8Start = poseFactory.of(82.4564, 15.8947, 270);
    private final Pose point8 = poseFactory.of(128.9538, 23.1118, 270);

    // Tune values
    private static final int BLUE_PIPELINE = 3;
    private static final double FALLBACK_VELOCITY = 1450;  // used when the Limelight can't see the hive
    private static final double INTAKE_DWELL = 1.5;        // seconds to sit at the wall with intake running

    // Max seconds per path. If the robot can't reach the end (wall, pollen in the way)
    // isBusy() never goes false, so move on after this instead of getting stuck.
    private static final double PATH1_TIMEOUT = 1.5;
    private static final double PATH2_TIMEOUT = 2.0;
    private static final double PATH3_TIMEOUT = 2.0;
    private static final double PATH4_TIMEOUT = 1.5;
    private static final double PATH5_TIMEOUT = 1.5;
    private static final double PATH6_TIMEOUT = 1.5;
    private static final double PATH7_TIMEOUT = 1.5;
    private static final double PATH8_TIMEOUT = 3.0;

    private Follower follower;
    private final ElapsedTime timer = new ElapsedTime();
    private final ElapsedTime pathTimer = new ElapsedTime();
    private double pathTimeout = 0;

    private Path firstPath;
    private Path secondPath;
    private Path thirdPath;
    private Path fourthPath;
    private Path fifthPath;
    private Path sixthPath;
    private Path seventhPath;
    private Path eighthPath;

    private int pathState = 0;

    private List<LynxModule> allHubs;

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

    public Path path7() {
        return Paths.line(point7Start, point7).linear(point7Start, point7);
    }

    public Path path8() {
        return Paths.line(point8Start, point8).linear(point8Start, point8);
    }

    private void followPath(Path path, double timeout) {
        follower.follow(path);
        pathTimeout = timeout;
        pathTimer.reset();
    }

    private boolean pathDone() {
        return !follower.isBusy() || pathTimer.seconds() >= pathTimeout;
    }

    @Override
    public void init() {

        // Utilities must init before the follower so Pedro's motor directions win
        utilities.init(hardwareMap, "auto");
        utilities.SetPipeline(BLUE_PIPELINE);
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

        firstPath = path1();
        secondPath = path2();
        thirdPath = path3();
        fourthPath = path4();
        fifthPath = path5();
        sixthPath = path6();
        seventhPath = path7();
        eighthPath = path8();
    }

    @Override
    public void start() {
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
                    followPath(firstPath, PATH1_TIMEOUT);
                    pathState = 2;
                }
                break;

            case 2: // backed off the wall, line up over the pollen
                if (pathDone()) {
                    followPath(secondPath, PATH2_TIMEOUT);
                    pathState = 3;
                }
                break;

            case 3: // lined up over the pollen, drive up to the wall with intake on
                if (pathDone()) {
                    utilities.Setintake(0.7, 0.4);
                    followPath(thirdPath, PATH3_TIMEOUT);
                    pathState = 4;
                }
                break;

            case 4:
                if (pathDone()) {
                    timer.reset();
                    pathState = 5;
                }
                break;

            case 5: // let the intake finish grabbing pollen, keep it running while backing off
                if (timer.seconds() >= INTAKE_DWELL) {
                    followPath(fourthPath, PATH4_TIMEOUT);
                    pathState = 6;
                }
                break;

            case 6: // move over to the throw lane
                if (pathDone()) {
                    utilities.Setintake(0.0, 0.0);
                    utilities.SetShooterVelocity(FALLBACK_VELOCITY); // spin up early to save time
                    followPath(fifthPath, PATH5_TIMEOUT);
                    pathState = 7;
                }
                break;

            case 7: // drive up to the throw spot near the wall
                if (pathDone()) {
                    followPath(sixthPath, PATH6_TIMEOUT);
                    pathState = 8;
                }
                break;

            case 8: // only start the throw once the robot has stopped at the throw spot
                if (pathDone()) {
                    utilities.BeginShot(FALLBACK_VELOCITY);
                    pathState = 9;
                }
                break;

            case 9: // throw grabbed pollen
                if (utilities.UpdateShot()) {
                    followPath(seventhPath, PATH7_TIMEOUT);
                    pathState = 10;
                }
                break;

            case 10: // backed off the wall, go park
                if (pathDone()) {
                    followPath(eighthPath, PATH8_TIMEOUT);
                    pathState = 11;
                }
                break;

            case 11: // park
                if (pathDone()) {
                    utilities.StopShooting();
                    pathState = 12;
                }
                break;

            case 12:
                break;
        }

        telemetry.addData("Path State", pathState);
        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData("Commanded Velocity", utilities.GetShotVelocity());
        telemetry.addData("Shooter1 Velocity", utilities.getTurretVelocity());
        telemetry.addData("Shooter2 Velocity", utilities.getShooter2Velocity());
        telemetry.addData("Pollen Thrown", utilities.GetShotCount());
        telemetry.addData("Slinger Up", utilities.SlingerUp());
        telemetry.addData("Runtime", getRuntime());
        telemetry.update();
    }

    @Override
    public void stop() {
        utilities.StopShooting();
    }
}
