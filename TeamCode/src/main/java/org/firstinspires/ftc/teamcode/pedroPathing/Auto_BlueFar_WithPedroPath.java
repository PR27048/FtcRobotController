package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous
public class Auto_BlueFar_WithPedroPath extends OpMode {

    private Follower follower;
    private Timer pathTimer, opModeTimer;
    Auto_ServiceHelper_WithPedroPath Helper = new Auto_ServiceHelper_WithPedroPath();

    public enum PathState {
        SHOOT_PRELOAD,
        DRIVE_INTAKELOADINGZONE,
        DRIVE_LOADINGZONE_BACK,
        DRIVE_LOADINGZONE_FORWARD,
        DRIVE_LAUNCH,
        SHOOT_PRELOAD_1,
        DRIVE_FIRSTSPIKE,
        DRIVE_INTAKE_FIRSTSPIKE,
        SHOOT_PRELOAD_2,
        NONE
    }

    private PathState pathState;

    // Poses
    private final Pose StartC1 = new Pose(56, 9.5, Math.toRadians(180));
    private final Pose CollectLoadingZone = new Pose(10.6, 9.5, Math.toRadians(180));
    private final Pose CollectLoadingZoneBack = new Pose(18, 9.5, Math.toRadians(180));
    private final Pose CollectFirstSpike = new Pose(56, 35.57, Math.toRadians(180));
    private final Pose IntakeFullFirstSpike = new Pose(10.01, 35.68, Math.toRadians(180));

    // Paths
    private PathChain Intake_loadingzone;
    private PathChain Intake_loadingzoneback;
    private PathChain Intake_loadingzoneforward;
    private PathChain Launch_Artifacts;
    private PathChain Drive_firstspike;
    private PathChain Intake_FirstSpike;
    private PathChain Return_To_Start;

    @Override
    public void init() {

        Helper.init(hardwareMap, "FALSE");

        pathTimer = new Timer();
        opModeTimer = new Timer();

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setPose(StartC1);

        pathState = PathState.SHOOT_PRELOAD;
        pathTimer.resetTimer();

        Helper.lifthood();
    }

    @Override
    public void start() {
        opModeTimer.resetTimer();
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {

        follower.update();
        statePathUpdate();
        Helper.AutoIntake();

        // Shooting logic
        switch (pathState) {
            case SHOOT_PRELOAD:
            case SHOOT_PRELOAD_1:
            case SHOOT_PRELOAD_2:
                handleShooting();
                break;

            default:
               // Helper.SetTurretOFF();
                Helper.PauseTrack();
                break;
        }

        telemetry.addData("State", pathState);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());
        telemetry.update();
    }

    private boolean turretStarted = false;

    private void handleShooting() {
        // Start turret only once per shooting state
        if (!turretStarted) {
            Helper.StartTurret(1400);
            turretStarted = true;
        }

        // Only shoot if turret is at speed AND robot is stationary
        if (/*Helper.isTurretAtSpeed(1400) &&*/ !follower.isBusy()) {
            Helper.AutoShoot(1400);
            // Optional: Helper.AutoTrack(1);
        }
    }

    private void buildPaths() {

        Intake_loadingzone = follower.pathBuilder()
                .addPath(new BezierLine(StartC1, CollectLoadingZone))
                .setLinearHeadingInterpolation(StartC1.getHeading(), CollectLoadingZone.getHeading())
                .build();

        Intake_loadingzoneback = follower.pathBuilder()
                .addPath(new BezierLine(CollectLoadingZone, CollectLoadingZoneBack))
                .setLinearHeadingInterpolation(CollectLoadingZone.getHeading(), CollectLoadingZoneBack.getHeading())
                .build();

        Intake_loadingzoneforward = follower.pathBuilder()
                .addPath(new BezierLine(CollectLoadingZoneBack, CollectLoadingZone))
                .setLinearHeadingInterpolation(CollectLoadingZoneBack.getHeading(), CollectLoadingZone.getHeading())
                .build();

        Launch_Artifacts = follower.pathBuilder()
                .addPath(new BezierLine(CollectLoadingZone, StartC1))
                .setLinearHeadingInterpolation(CollectLoadingZone.getHeading(), StartC1.getHeading())
                .build();

        Drive_firstspike = follower.pathBuilder()
                .addPath(new BezierLine(StartC1, CollectFirstSpike))
                .setLinearHeadingInterpolation(StartC1.getHeading(), CollectFirstSpike.getHeading())
                .build();

        Intake_FirstSpike = follower.pathBuilder()
                .addPath(new BezierLine(CollectFirstSpike, IntakeFullFirstSpike))
                .setLinearHeadingInterpolation(CollectFirstSpike.getHeading(), IntakeFullFirstSpike.getHeading())
                .build();

        Return_To_Start = follower.pathBuilder()
                .addPath(new BezierLine(IntakeFullFirstSpike, StartC1))
                .setLinearHeadingInterpolation(IntakeFullFirstSpike.getHeading(), StartC1.getHeading())
                .build();
    }

    private void statePathUpdate() {

        switch (pathState) {

            case SHOOT_PRELOAD:
                // Shoot for 3 seconds before moving
                if (pathTimer.getElapsedTimeSeconds() > 3) {
                    follower.followPath(Intake_loadingzone);
                    setPathState(PathState.DRIVE_INTAKELOADINGZONE);
                }
                break;

            case DRIVE_INTAKELOADINGZONE:
                if (!follower.isBusy()) {
                    follower.followPath(Intake_loadingzoneback);
                    setPathState(PathState.DRIVE_LOADINGZONE_BACK);
                }
                break;

            case DRIVE_LOADINGZONE_BACK:
                if (!follower.isBusy()) {
                    follower.followPath(Intake_loadingzoneforward);
                    setPathState(PathState.DRIVE_LOADINGZONE_FORWARD);
                }
                break;

            case DRIVE_LOADINGZONE_FORWARD:
                if (!follower.isBusy()) {
                    follower.followPath(Launch_Artifacts);
                    setPathState(PathState.SHOOT_PRELOAD_1);
                }
                break;

            case SHOOT_PRELOAD_1:
                if (pathTimer.getElapsedTimeSeconds() > 3) {
                    setPathState(PathState.DRIVE_LAUNCH);
                }
                break;

            case DRIVE_LAUNCH:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1) {
                    follower.followPath(Drive_firstspike);
                    setPathState(PathState.DRIVE_FIRSTSPIKE);
                }
                break;

            case DRIVE_FIRSTSPIKE:
                if (!follower.isBusy()) {
                    follower.followPath(Intake_FirstSpike);
                    setPathState(PathState.DRIVE_INTAKE_FIRSTSPIKE);
                }
                break;

            case DRIVE_INTAKE_FIRSTSPIKE:
                if (!follower.isBusy()) {
                    follower.followPath(Return_To_Start);
                    setPathState(PathState.SHOOT_PRELOAD_2);
                }
                break;

            case SHOOT_PRELOAD_2:
                if (pathTimer.getElapsedTimeSeconds() > 3) {
                    setPathState(PathState.NONE);
                }
                break;

            case NONE:
                telemetry.addLine("Auto Complete");
                break;
        }
    }

    private void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }
}