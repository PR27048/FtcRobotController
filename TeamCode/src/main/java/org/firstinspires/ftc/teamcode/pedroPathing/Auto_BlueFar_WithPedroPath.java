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

@Autonomous(name="Auto_BlueFar_WithPedroPath")
public class Auto_BlueFar_WithPedroPath extends OpMode {

    private Follower follower;
    private Timer pathTimer, opModeTimer;

    private DcMotor ServoConFront;
    private DcMotorEx MotorFeeder;

    Auto_ServiceHelper_WithPedroPath Helper = new Auto_ServiceHelper_WithPedroPath();

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
        DRIVE_AWAY_FROM_SHOOT_LINE,

        NONE
    }

    private PathState pathState;

    // ================= BLUE SIDE POSES =================
    private final Pose StartC1 = new Pose(55, 9.5, Math.toRadians(180));
    private final Pose CollectLoadingZone = new Pose(10.6, 9.5, Math.toRadians(180));
    private final Pose CollectLoadingZoneBack = new Pose(18, 9.5, Math.toRadians(180));

    private final Pose CollectFirstSpike = new Pose(56, 35.57, Math.toRadians(180));
    private final Pose IntakeFullFirstSpike = new Pose(10.01, 35.57, Math.toRadians(180));

    private final Pose SecondSpike = new Pose(56, 57.6, Math.toRadians(180));
    private final Pose CollectSecondSpike = new Pose(16.06, 57.3, Math.toRadians(180));

    //private final Pose NearShootPositionForSecondSpike = new Pose(61.187, 101.11835334476848, Math.toRadians(130)); // mirrored from Red
    private final Pose NearShootPositionForSecondSpike = new Pose(55, 78.2, Math.toRadians(117));
    //private final Pose MovefromShootLine = new Pose(42.8, 76.3, Math.toRadians(130));
    private final Pose MovefromShootLine = new Pose(50.354766031195844, 73.29480069324089, Math.toRadians(136));
    private final Pose ThirdSpike = new Pose(56.47, 84.41, Math.toRadians(180));
    private final Pose CollectThirdSpike = new Pose(16.20, 83.99, Math.toRadians(180));

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
    private PathChain Launch_NearSecondSpike;

    private PathChain Drive_ThirdSpike;
    private PathChain Intake_ThirdSpike;

    private PathChain MovefromShootLinePath;

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
        Helper.StartTurret(1320);
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

       // Helper.StartTurret(1320);
        Helper.AutoTrack(1); // Blue side

        switch (pathState) {

            case SHOOT_PRELOAD:
            case SHOOT_PRELOAD_1:
            case SHOOT_PRELOAD_2:
            case DRIVE_AWAY_FROM_SHOOT_LINE:
               // if (!follower.isBusy()) {  // commented per chatgpt
                    MotorFeeder.setPower(-1.0);
                    ServoConFront.setPower(-1.0);
                    Helper.AutoTrack(1);
              //  }
                break;

            default:
                MotorFeeder.setPower(1.0);
                ServoConFront.setPower(0.6);
                Helper.AutoTrack(1);
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

        Launch_NearSecondSpike = follower.pathBuilder()
                .addPath(new BezierLine(CollectSecondSpike, NearShootPositionForSecondSpike))
                .setLinearHeadingInterpolation(CollectSecondSpike.getHeading(), NearShootPositionForSecondSpike.getHeading())
                .build();

        MovefromShootLinePath = follower.pathBuilder()
                .addPath(new BezierLine(NearShootPositionForSecondSpike, MovefromShootLine))
                .setLinearHeadingInterpolation(NearShootPositionForSecondSpike.getHeading(), MovefromShootLine.getHeading())
                .build();
    }

    private void statePathUpdate() {

        switch (pathState) {

            case WAIT_BEFORE_START:
                if (Helper.isTurretAtSpeed(1320) && pathTimer.getElapsedTimeSeconds() > 2) {
                    setPathState(PathState.SHOOT_PRELOAD);
                }
                break;

            case SHOOT_PRELOAD:
                if (pathTimer.getElapsedTimeSeconds() > 2 && Helper.isTurretAtSpeed(1320)) {
                    follower.followPath(Intake_loadingzone);
                    setPathState(PathState.DRIVE_INTAKELOADINGZONE);
                }
                break;

            case DRIVE_INTAKELOADINGZONE:
                if (!follower.isBusy()) {
                    follower.followPath(Intake_loadingzoneback);
                   // setPathState(PathState.DRIVE_LOADINGZONE_BACK); // to see if we can save this time
                    setPathState(PathState.DRIVE_LOADINGZONE_FORWARD);
                }
                break;

//Not doing double load in loading zone
           /* case DRIVE_LOADINGZONE_BACK:
                if (!follower.isBusy()) {
                    follower.followPath(Intake_loadingzoneforward);
                    setPathState(PathState.DRIVE_LOADINGZONE_FORWARD);
                }
                break;*/

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

            case SHOOT_PRELOAD_1:
                if (pathTimer.getElapsedTimeSeconds() > 2.5) {
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

            case SHOOT_PRELOAD_2:
                if (pathTimer.getElapsedTimeSeconds() > 2.5) {
                    follower.followPath(Drive_SecondSpike);
                    setPathState(PathState.DRIVE_SECOND_SPIKE);
                    Helper.StartTurret(1020); // turret speed for shooting
                }
                break;

            case DRIVE_SECOND_SPIKE:
                if (!follower.isBusy()) {
                    follower.followPath(Intake_SecondSpike);
                    setPathState(PathState.DRIVE_INTAKE_SECOND_SPIKE);
                }
                break;

            case DRIVE_INTAKE_SECOND_SPIKE:
               // Helper.StartTurret(1020); // turret speed for shooting - moved up

                if (!follower.isBusy() && (Helper.isTurretAtSpeed(1020) && pathTimer.getElapsedTimeSeconds() > 3)) {
                    telemetry.addLine("velocity" + Helper.isTurretAtSpeed(1020));
                    follower.followPath(Launch_NearSecondSpike);
                    Helper.lowerhood();
                    setPathState(PathState.DRIVE_AWAY_FROM_SHOOT_LINE);

                }
                break;

            case DRIVE_AWAY_FROM_SHOOT_LINE:
                telemetry.addLine("velocity next" + Helper.isTurretAtSpeed(1020));
                if (!follower.isBusy() && (Helper.isTurretAtSpeed(1020) && pathTimer.getElapsedTimeSeconds() > 3)) {
                    follower.followPath(MovefromShootLinePath);
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