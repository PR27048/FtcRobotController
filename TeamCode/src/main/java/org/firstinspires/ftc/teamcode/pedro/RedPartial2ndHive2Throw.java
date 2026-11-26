package org.firstinspires.ftc.teamcode.pedro;

import org.firstinspires.ftc.teamcode.Teleops.Utilities;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.List;

/*
 * Red alliance, start position at the top wall (2nd hive side), two throws.
 * Heading is the intake (front) direction. At 90 the intake faces the top wall
 * and the shooter (back) faces the hive.
 *
 *  1. path1 - back off the wall to the throw spot (shooter spins up)
 *  2. Throw preloaded pollen into the hive
 *  3. path2 - line up under the pollen at the top wall
 *  4. path3 - drive into the wall with intake running, grab pollen
 *  5. path4 - back off the wall
 *  6. path5 - drive to the throw spot and throw
 *  7. path6 - park
 */
@Autonomous(name = "RedPartial2ndHive2Throw")
public class RedPartial2ndHive2Throw extends OpMode {

    private final Utilities utilities = new Utilities();

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(59.5006, 133.655, 90);
    private final Pose point1 = poseFactory.of(59.8691, 124.2533, 90);
    private final Pose point2Start = poseFactory.of(59.8691, 124.2533, 90);
    private final Pose point2 = poseFactory.of(47.9312, 114.0549, 90);
    private final Pose point3Start = poseFactory.of(47.9312, 114.0549, 90);
    private final Pose point3 = poseFactory.of(47.1921, 128.329, 90);
    private final Pose point4Start = poseFactory.of(47.1921, 128.329, 90);
    private final Pose point4 = poseFactory.of(47.4653, 119.9219, 90);
    private final Pose point5Start = poseFactory.of(47.4653, 119.9219, 90);
    private final Pose point5 = poseFactory.of(59.4142, 125.7162, 90);
    private final Pose point6Start = poseFactory.of(59.4142, 125.7162, 90);
    private final Pose point6 = poseFactory.of(59.4241, 131.968, 90);
    private final Pose point7Start = poseFactory.of(59.4241, 131.968, 90);
    private final Pose point7 = poseFactory.of(59.0436, 125.6053, 90);
    private final Pose point8Start = poseFactory.of(59.0436, 125.6053, 90);
    private final Pose point8 = poseFactory.of(12.5462, 118.3882, 90);

    // Tune values
    private static final int RED_PIPELINE = 2;
    private static final double FALLBACK_VELOCITY = 1450; //1185;  // used when the Limelight can't see the hive (RedAutoTimed value)
    private static final double SPINUP_TIMEOUT = 1.5;      // seconds to wait for shooter to reach speed
    private static final double FEED_TIME = 3; //2.5;           // seconds to feed pollen into the shooter
    private static final double INTAKE_DWELL = 1.0;        // seconds to sit at the wall with intake running

    private Follower follower;
    private final ElapsedTime timer = new ElapsedTime();
    private final ElapsedTime shotTimer = new ElapsedTime();

    private Path firstPath;
    private Path secondPath;
    private Path thirdPath;
    private Path fourthPath;
    private Path fifthPath;
    private Path sixthPath,seventhPath,eighthPath;

    private int pathState = 0;
    private double velocity = FALLBACK_VELOCITY;
    private boolean feeding = false;

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


    private void updateShooterVelocity() {
        velocity = 1450;
        utilities.SetShooterVelocity(velocity);
    }

    private void beginShot() {
        velocity = FALLBACK_VELOCITY;
        feeding = false;
        shotTimer.reset();
      }

    // Call every loop while shooting, returns true once all pollen has been thrown
    private boolean updateShot() {
        updateShooterVelocity();
        utilities.SetGate(0.69);

        if (!feeding) {
            if (utilities.isTurretAtSpeed(velocity) || shotTimer.seconds() >= SPINUP_TIMEOUT) {
                feeding = true;
                shotTimer.reset();
            }
            return false;
        }

        if (utilities.SlingerUp() && utilities.isTurretAtSpeed(velocity)) {
            utilities.Setintake(0.7, 0.4); //shooting speeed adjust
            utilities.SetFeeder(1.0);
        } else {
            utilities.Setintake(0.0, 0.0);
        }

        if (shotTimer.seconds() >= FEED_TIME) {
            stopShooting();
            return true;
        }
        return false;
    }

    private void stopShooting() {
        utilities.Setintake(0.0, 0.0);
        utilities.SetFeeder(0.0);
        utilities.SetGate(0.07);
        utilities.SetShooterVelocity(0);
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
        // Faster loop for the follower
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL); //AUTO
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
        utilities.SetShooterVelocity(FALLBACK_VELOCITY); // spin up while backing off the wall
        beginShot();
        pathState = 1;
    }

    @Override
    public void loop() {
        // 1. Refresh all hub sensor/encoder data ONCE at the start of the loop
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }

        follower.update();

        switch (pathState) {
            case 1:
                if (updateShot()) {
                    follower.follow(firstPath);
                    pathState = 2;
                }
                break;

            case 2:
                if (!follower.isBusy()) {// if (updateShot()) {
                    follower.follow(secondPath);
                    pathState = 3;
                }
                break;

            case 3: // lined up under the pollen, drive into the wall with intake on
                if (!follower.isBusy()) {
                    utilities.Setintake(0.7, 0.6);
                    follower.follow(thirdPath);
                    pathState = 4;
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    timer.reset();
                    pathState = 5;
                }
                break;

            case 5: // let the intake finish grabbing pollen, keep it running while backing off
                if (timer.seconds() >= INTAKE_DWELL) {
                    follower.follow(fourthPath);
                    pathState = 6;
                }
                break;

            case 6:
                if (!follower.isBusy()) {
                    utilities.Setintake(0.0, 0.0); //uncomment both line
                    utilities.SetShooterVelocity(FALLBACK_VELOCITY); // spin up early to save time
                    follower.follow(fifthPath);
                    pathState = 7;
                }
                break;

            case 7:
                if (!follower.isBusy()) {
                    beginShot();
                    pathState = 8;
                }
                break;

            case 8: // throw grabbed pollen
                if (updateShot()) { //uncomment
                    follower.follow(sixthPath);
                    pathState = 9;
                }
                break;

            case 9: // park
                if (!follower.isBusy()) {
                    stopShooting();
                    pathState = 10;
                }
                break;

            case 10:
                break;
        }

        telemetry.addData("Path State", pathState);
        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData("Commanded Velocity", velocity);
        telemetry.addData("Actual Velocity", utilities.getTurretVelocity());
        telemetry.addData("Runtime", getRuntime());
        telemetry.update();
    }

    @Override
    public void stop() {
        stopShooting();
    }
}