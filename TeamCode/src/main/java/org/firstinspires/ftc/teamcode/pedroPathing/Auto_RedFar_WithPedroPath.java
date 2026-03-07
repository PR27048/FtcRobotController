package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous(name="Auto_RedFar_WithPedroPath")
public class Auto_RedFar_WithPedroPath extends OpMode {

    private Follower follower;
    private Timer pathTimer, opModeTimer;

    private DcMotor ServoConFront;

    Auto_ServiceHelper_WithPedroPath Helper = new Auto_ServiceHelper_WithPedroPath();

    private DcMotorEx MotorFeeder;

    // ================= STATES =================

    public enum PathState {

        WAIT_BEFORE_START,
        SHOOT_PRELOAD,

        DRIVE_INTAKELOADINGZONE,
        DRIVE_LOADINGZONE_BACK,
        DRIVE_LOADINGZONE_FORWARD,
        DRIVE_LAUNCH,

        SHOOT_PRELOAD_1,
        DRIVE_FIRSTSPIKE,
        DRIVE_INTAKE_FIRSTSPIKE,
        RETURN_FROM_FIRST,

        SHOOT_PRELOAD_2,
        DRIVE_SECOND_SPIKE,
        DRIVE_INTAKE_SECOND_SPIKE,
        RETURN_FROM_SECOND,

        SHOOT_PRELOAD_3,
        DRIVE_THIRD_SPIKE,
        DRIVE_INTAKE_THIRD_SPIKE,
        DRIVE_TO_FINAL_SHOOT,

        FINAL_SHOOT,
        START_DRIVE_OFF_LAUNCHLINE,
        DRIVE_OFF_LAUNCHLINE,

        NONE
    }

    private PathState pathState;

    // ================= POSES =================

    private final Pose StartC1 = new Pose(56, 134.5, Math.toRadians(180));

    private final Pose CollectLoadingZone = new Pose(10.6, 134.5, Math.toRadians(180));
    private final Pose CollectLoadingZoneBack = new Pose(18, 134.5, Math.toRadians(180));

    private final Pose CollectFirstSpike = new Pose(56, 108.43, Math.toRadians(180));
    private final Pose IntakeFullFirstSpike = new Pose(10.01, 108.43, Math.toRadians(180));

    private final Pose SecondSpike = new Pose(56, 82.74, Math.toRadians(180));
    private final Pose CollectSecondSpike = new Pose(16.06, 82.74, Math.toRadians(180));

    private final Pose ThirdSpike = new Pose(56.47, 59.59, Math.toRadians(180));
    private final Pose CollectThirdSpike = new Pose(16.20, 60.01, Math.toRadians(180));

    private final Pose NearShootPositionForThirdSpike = new Pose(56.47, 59.59, Math.toRadians(-130));
    private final Pose MovefromShootLine = new Pose(42.8, 67.7, Math.toRadians(-130));
    // ================= PATHS =================

    private PathChain Intake_loadingzone;
    private PathChain Intake_loadingzoneback;
    private PathChain Intake_loadingzoneforward;
    private PathChain Launch_Artifacts;
    private PathChain Drive_firstspike;
    private PathChain Intake_FirstSpike;
    private PathChain Return_To_Start;
    private PathChain Drive_SecondSpike;
    private PathChain Intake_SecondSpike;
    private PathChain Drive_ThirdSpike;
    private PathChain Intake_ThirdSpike;
    private PathChain Launch_NearThirdSpike;
    private PathChain MovefromShootLinePath;

    // ================= INIT =================

    @Override
    public void init() {

        Helper.init(hardwareMap, "FALSE");
        MotorFeeder = hardwareMap.get(DcMotorEx.class, "motorizedtransfer");
        ServoConFront = hardwareMap.get(DcMotor.class, "servo_con_front_transfer");

        pathTimer = new Timer();
        opModeTimer = new Timer();

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setPose(StartC1);

        pathState = PathState.WAIT_BEFORE_START;
        pathTimer.resetTimer();

        Helper.lifthood();
    }

    @Override
    public void start() {
        opModeTimer.resetTimer();
        pathTimer.resetTimer();
    }

    // ================= LOOP =================

    @Override
    public void loop() {

        follower.update();
        statePathUpdate();

        Helper.AutoIntake();
        Helper.StartTurret(1380);
        Helper.AutoTrack(0);

        // ===== Shooter Safety Logic =====
        switch (pathState) {
            case SHOOT_PRELOAD:
            case SHOOT_PRELOAD_1:
            case SHOOT_PRELOAD_2:
            case SHOOT_PRELOAD_3:
            case FINAL_SHOOT:
                if (!follower.isBusy()) {
                    MotorFeeder.setPower(-1.0);
                    ServoConFront.setPower(-1.0);

                    Helper.AutoTrack(0);
                }
                break;

            default:
                MotorFeeder.setPower(1.0);
                ServoConFront.setPower(0.6);

                Helper.AutoTrack(0);
                break;
        }

        telemetry.addData("State", pathState);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());
        telemetry.update();
    }

    // ================= PATH BUILDER =================

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
                .addPath(new BezierLine(CollectThirdSpike, StartC1))
                .setLinearHeadingInterpolation(CollectThirdSpike.getHeading(), StartC1.getHeading())

                .build();

        MovefromShootLinePath = follower.pathBuilder()
                .addPath(new BezierLine(StartC1, MovefromShootLine))
                .setLinearHeadingInterpolation(StartC1.getHeading(), MovefromShootLine.getHeading())

                .build();
    }

    // ================= STATE MACHINE =================

    private void statePathUpdate() {

        switch (pathState) {

            // ================= START: WAIT FOR TURRET SPEED =================

            case WAIT_BEFORE_START:
                //Helper.StartTurret(1370);

                if (Helper.isTurretAtSpeed(1380) && pathTimer.getElapsedTimeSeconds() > 3) {
                    setPathState(PathState.SHOOT_PRELOAD);
                }
                break;

            case SHOOT_PRELOAD:
                if (pathTimer.getElapsedTimeSeconds() > 3 && Helper.isTurretAtSpeed(1380)) {
                    follower.followPath(Intake_loadingzone);
                    setPathState(PathState.DRIVE_INTAKELOADINGZONE);
                }
                break;

            // ================= LOADING ZONE =================

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
                    setPathState(PathState.DRIVE_LAUNCH);
                }
                break;

            case DRIVE_LAUNCH:
                if (!follower.isBusy()) {
                    setPathState(PathState.SHOOT_PRELOAD_1);
                }
                break;

            // ================= FIRST SPIKE =================

            case SHOOT_PRELOAD_1:
                if (pathTimer.getElapsedTimeSeconds() > 3) {
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
                    setPathState(PathState.RETURN_FROM_FIRST);
                }
                break;

            case RETURN_FROM_FIRST:
                if (!follower.isBusy()) {
                    setPathState(PathState.SHOOT_PRELOAD_2);
                }
                break;

            // ================= SECOND SPIKE =================

            case SHOOT_PRELOAD_2:
                if (pathTimer.getElapsedTimeSeconds() > 4) {
                    follower.followPath(Drive_SecondSpike);
                    setPathState(PathState.DRIVE_SECOND_SPIKE);
                }
                break;

            case DRIVE_SECOND_SPIKE:
                if (!follower.isBusy()) {
                    follower.followPath(Intake_SecondSpike);
                    setPathState(PathState.DRIVE_INTAKE_SECOND_SPIKE);
                }
                break;

            case DRIVE_INTAKE_SECOND_SPIKE:
                if (!follower.isBusy()) {
                    follower.followPath(Return_To_Start);
                    setPathState(PathState.RETURN_FROM_SECOND);
                }
                break;

            case RETURN_FROM_SECOND:
                if (!follower.isBusy()) {
                    setPathState(PathState.SHOOT_PRELOAD_3);
                }
                break;

            // ================= THIRD SPIKE =================

            case SHOOT_PRELOAD_3:
                if (pathTimer.getElapsedTimeSeconds() > 3) {
                    follower.followPath(MovefromShootLinePath);
                    setPathState(PathState.START_DRIVE_OFF_LAUNCHLINE);
                }
                break;


            case START_DRIVE_OFF_LAUNCHLINE:
                setPathState(PathState.DRIVE_OFF_LAUNCHLINE);
                break;

            case DRIVE_OFF_LAUNCHLINE:
                if (!follower.isBusy()) {
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