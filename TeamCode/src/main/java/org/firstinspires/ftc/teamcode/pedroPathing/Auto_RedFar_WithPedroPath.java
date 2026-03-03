package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous(name="Auto_RedFar_WithPedroPath")
public class Auto_RedFar_WithPedroPath extends OpMode {

    private Follower follower;
    private Timer pathTimer, opModeTimer;
    Auto_ServiceHelper_WithPedroPath Helper = new Auto_ServiceHelper_WithPedroPath();

    private DcMotorEx MotorFeeder;

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
        DRIVE_SECOND_SPIKE,
        DRIVE_INTAKE_SECOND_SPIKE,
        DRIVE_Third_SPIKE,

        DRIVE_INTAKE_THIRD_SPIKE,
        SHOOT_PRELOAD_3,
        DRIVE_OFF_LAUNCHLINE,
        NONE
    }

    private PathState pathState;

    //  RED FAR POSES (Mirrored X = 144 - BlueX)
    private final Pose StartC1 = new Pose(88, 9.5, Math.toRadians(0));
    private final Pose CollectLoadingZone = new Pose(133.4, 9.5, Math.toRadians(0));
    private final Pose CollectLoadingZoneBack = new Pose(126, 9.5, Math.toRadians(0));

    private final Pose CollectFirstSpike = new Pose(88, 35.57, Math.toRadians(0));
    private final Pose IntakeFullFirstSpike = new Pose(133.99, 35.68, Math.toRadians(0));

    private final Pose SecondSpike = new Pose(88, 61.26, Math.toRadians(0));
    private final Pose CollectSecondSpike = new Pose(127.94, 61.26, Math.toRadians(0));
    private final Pose ThirdSpike = new Pose(87.53032928942808, 84.41941074523398, Math.toRadians(0));
    private final Pose CollectThirdSpike = new Pose(127.80415944540732, 83.99480069324089, Math.toRadians(0));
    private final Pose NearShootPositionForThirdSpike = new Pose(87.53032928942808, 84.41941074523398, Math.toRadians(50));
    private final Pose MovefromShootLine = new Pose(101.2, 76.3, Math.toRadians(50));
    // Paths
    private PathChain Intake_loadingzone;
    private PathChain Intake_loadingzoneback;
    private PathChain Intake_loadingzoneforward;
    private PathChain Launch_Artifacts;
    private PathChain Drive_firstspike;
    private PathChain Intake_FirstSpike;
    private PathChain Return_To_Start;
    private PathChain Drive_SecondSpike, Drive_ThirdSpike;
    private PathChain Intake_SecondSpike, Intake_ThirdSpike, Launch_NearThirdSpike, MovefromShootLinePath;

    @Override
    public void init() {

        Helper.init(hardwareMap, "FALSE");
        MotorFeeder = hardwareMap.get(DcMotorEx.class, "motorizedtransfer");

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
        Helper.StartTurret(1370);

        switch (pathState) {
            case SHOOT_PRELOAD:
            case SHOOT_PRELOAD_1:
            case SHOOT_PRELOAD_2:
            case SHOOT_PRELOAD_3:
                MotorFeeder.setPower(-1.0);
                Helper.AutoTrack(1);
                break;

            default:
                MotorFeeder.setPower(1.0);
                Helper.PauseTrack();
                break;
        }

        telemetry.addData("State", pathState);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());
        telemetry.update();
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

        Drive_SecondSpike = follower.pathBuilder()
                .addPath(new BezierLine(StartC1, SecondSpike))
                .setLinearHeadingInterpolation(StartC1.getHeading(), SecondSpike.getHeading())
                .build();

        Intake_SecondSpike = follower.pathBuilder()
                .addPath(new BezierLine(SecondSpike, CollectSecondSpike))
                .setLinearHeadingInterpolation(SecondSpike.getHeading(), CollectSecondSpike.getHeading())
                .build();

        Drive_ThirdSpike = follower.pathBuilder()
                .addPath(new BezierLine(StartC1, ThirdSpike))
                .setLinearHeadingInterpolation(StartC1.getHeading(), ThirdSpike.getHeading())
                .build();

        Intake_ThirdSpike = follower.pathBuilder()
                .addPath(new BezierLine(ThirdSpike, CollectThirdSpike))
                .setLinearHeadingInterpolation(ThirdSpike.getHeading(), CollectThirdSpike.getHeading())
                .build();

        Launch_NearThirdSpike = follower.pathBuilder()
                .addPath(new BezierLine(CollectThirdSpike, NearShootPositionForThirdSpike))
                .setLinearHeadingInterpolation(CollectThirdSpike.getHeading(), NearShootPositionForThirdSpike.getHeading())
                .build();

        MovefromShootLinePath = follower.pathBuilder()
                .addPath(new BezierLine(NearShootPositionForThirdSpike, MovefromShootLine))
                .setLinearHeadingInterpolation(NearShootPositionForThirdSpike.getHeading(), MovefromShootLine.getHeading())
                .build();
    }

    // Your original state machine stays EXACTLY the same
    private void statePathUpdate() {
        boolean done = false;

        switch (pathState) {
            case SHOOT_PRELOAD:
                if (pathTimer.getElapsedTimeSeconds() > 4) {
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
                if (pathTimer.getElapsedTimeSeconds() > 4) {
                    setPathState(PathState.DRIVE_LAUNCH);
                }
                done = true;
                break;

            case DRIVE_LAUNCH:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1 && (done=true)) {
                    follower.followPath(Drive_firstspike);
                    setPathState(PathState.DRIVE_FIRSTSPIKE);
                }
                done = false;
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
                if (pathTimer.getElapsedTimeSeconds() > 4) {
                    follower.followPath(Drive_SecondSpike);
                    setPathState(PathState.DRIVE_SECOND_SPIKE);
                }
                done = true;
                break;

            case DRIVE_SECOND_SPIKE:
                if (!follower.isBusy()  && (done=true)) {
                    follower.followPath(Intake_SecondSpike);
                    setPathState(PathState.DRIVE_INTAKE_SECOND_SPIKE);
                }
                done = false;
                break;

            case DRIVE_INTAKE_SECOND_SPIKE:
                if (!follower.isBusy()) {
                    follower.followPath(Return_To_Start);
                    setPathState(PathState.SHOOT_PRELOAD_3);
                }
                break;

            case SHOOT_PRELOAD_3:
                if (pathTimer.getElapsedTimeSeconds() > 4) {
                    follower.followPath(Drive_ThirdSpike);
                    setPathState(PathState.DRIVE_Third_SPIKE);

                }
                done = true;
                break;
            case DRIVE_Third_SPIKE:
                if (!follower.isBusy()  && (done=true)) {
                    follower.followPath(Intake_ThirdSpike);
                    setPathState(PathState.DRIVE_INTAKE_THIRD_SPIKE);
                }
                done = false;
                break;
            case DRIVE_INTAKE_THIRD_SPIKE:
                if (!follower.isBusy()) {
                    follower.followPath(Launch_NearThirdSpike);
                    setPathState(PathState.DRIVE_OFF_LAUNCHLINE);
                }
                break;
            case DRIVE_OFF_LAUNCHLINE:
                if (pathTimer.getElapsedTimeSeconds() > 4) {
                    follower.followPath(MovefromShootLinePath);
                    setPathState(PathState.NONE);
                }
                break;
            case NONE:
                telemetry.addLine("Auto Complete");
                break;
            default:
                telemetry.addLine("No path selected");
                break;
        }
    }

    private void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }
}