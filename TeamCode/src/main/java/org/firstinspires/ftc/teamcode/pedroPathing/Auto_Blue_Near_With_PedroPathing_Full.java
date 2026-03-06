package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous
public class Auto_Blue_Near_With_PedroPathing_Full extends OpMode {

    Auto_ServiceHelper_WithPedroPath helper = new Auto_ServiceHelper_WithPedroPath();
    private DcMotorEx Turret, MotorFeeder;
    private Servo ServoConTurret;
    private DcMotor ServoConFront;

    private double TurretVelocity = 1020;

    private Follower follower;
    private Timer pathTimer, OpModeTimer;

    public enum PathState {
        DRIVE_STARTPOS_SHOOT_POS,
        SHOOT_PRELOAD_1,
        DRIVE_SHOOT_POS_FIRSTROW_INTAKE_SETUP,
        DRIVE_FIRSTROW_INTAKE_SETUP_INTAKE_FIRSTROW,
        DRIVE_INTAKE_FIRSTROW_GATE_SETUP,
        DRIVE_INTAKE_GATE_SETUP_OPEN_GATE,
        DRIVE_OPEN_GATE_SHOOT_POS,
        SHOOT_2,
        DRIVE_SHOOT_POS_SECOND_ROW_INTAKE_SETUP,
        DRIVE_SECOND_ROW_INTAKE_SETUP_INTAKE_SECOND_ROW,
        DRIVE_INTAKE_SECOND_ROW_AVOID_GATE,
        DRIVE_AVOID_GATE_SHOOT_POS,
        SHOOT_3,
        DRIVE_SHOOT_POS_THIRD_ROW_INTAKE_SETUP,
        DRIVE_THIRD_ROW_INTAKE_SETUP_INTAKE_THIRD_ROW,
        DRIVE_INTAKE_THIRD_ROW_SHOOT_POS,
        SHOOT_4,
        DRIVE_SHOOT_POS_GATE_SETUP
    }

    PathState pathState;
    private boolean pathStarted = false;

    // ================= BLUE POSES =================

    private final Pose startPose =
            new Pose(123.28729281767956, 21.43646408839778, Math.toRadians(-37));

    private final Pose shootPose =
            new Pose(91.80110497237568, 54.24033149171271, Math.toRadians(-47));

    private final Pose firstRowIntakeSetUpPose =
            new Pose(92.12154696132599, 60.80662983425415, Math.toRadians(0));

    private final Pose intakeFirstRowPose =
            new Pose(129.38674033149172, 60.86740331491714, Math.toRadians(0));

    private final Pose gateSetUpPose =
            new Pose(119.20994475138122, 72.98342541436465, Math.toRadians(0));

    private final Pose openGatePose =
            new Pose(128.80939226519337, 73.30939226519338, Math.toRadians(0));

    private final Pose secondRowIntakeSetUpPose =
            new Pose(95.3259668508287, 84.2099447513812, Math.toRadians(0));

    private final Pose intakeSecondRowPose =
            new Pose(134.98342541436463, 85.31491712707183, Math.toRadians(0));

    private final Pose avoidGatePose =
            new Pose(119.01104972375691, 85.03867403314918, Math.toRadians(0));

    private final Pose thirdRowIntakeSetUpPose =
            new Pose(96.62430939226519, 108.08287292817678, Math.toRadians(0));

    private final Pose intakeThirdRowPose =
            new Pose(135.41988950276243, 108.24309392265193, Math.toRadians(0));

    // =================================================

    private PathChain driveStartPosShootPos,
            driveShootPosFirstRowIntakeSetUpPos,
            driveFirstRowIntakeSetUpPosIntakeFirstRowPos,
            driveIntakeFirstRowPosGateSetUpPos,
            driveGateSetUpPosOpenGatePos,
            driveOpenGatePosShootPos,
            driveShootPosSecondRowIntakeSetUp,
            driveSecondRowIntakeSetUpIntakeSecondRow,
            driveIntakeSecondRowAvoidGate,
            driveAvoidGateShootPos,
            driveShootPosThirdRowIntakeSetUp,
            driveThirdRowIntakeSetUpIntakeThirdRow,
            driveIntakeThirdRowShootPos,
            driveShootPosGateSetUp;

    public void buildPaths() {

        driveStartPosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        driveShootPosFirstRowIntakeSetUpPos = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, firstRowIntakeSetUpPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), firstRowIntakeSetUpPose.getHeading())
                .build();

        driveFirstRowIntakeSetUpPosIntakeFirstRowPos = follower.pathBuilder()
                .addPath(new BezierLine(firstRowIntakeSetUpPose, intakeFirstRowPose))
                .setLinearHeadingInterpolation(firstRowIntakeSetUpPose.getHeading(), intakeFirstRowPose.getHeading())
                .build();

        driveIntakeFirstRowPosGateSetUpPos = follower.pathBuilder()
                .addPath(new BezierLine(intakeFirstRowPose, gateSetUpPose))
                .setLinearHeadingInterpolation(intakeFirstRowPose.getHeading(), gateSetUpPose.getHeading())
                .build();

        driveGateSetUpPosOpenGatePos = follower.pathBuilder()
                .addPath(new BezierLine(gateSetUpPose, openGatePose))
                .setLinearHeadingInterpolation(gateSetUpPose.getHeading(), openGatePose.getHeading())
                .build();

        driveOpenGatePosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(openGatePose, shootPose))
                .setLinearHeadingInterpolation(openGatePose.getHeading(), shootPose.getHeading())
                .build();

        driveShootPosSecondRowIntakeSetUp = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, secondRowIntakeSetUpPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), secondRowIntakeSetUpPose.getHeading())
                .build();

        driveSecondRowIntakeSetUpIntakeSecondRow = follower.pathBuilder()
                .addPath(new BezierLine(secondRowIntakeSetUpPose, intakeSecondRowPose))
                .setLinearHeadingInterpolation(secondRowIntakeSetUpPose.getHeading(), intakeSecondRowPose.getHeading())
                .build();

        driveIntakeSecondRowAvoidGate = follower.pathBuilder()
                .addPath(new BezierLine(intakeSecondRowPose, avoidGatePose))
                .setLinearHeadingInterpolation(intakeSecondRowPose.getHeading(), avoidGatePose.getHeading())
                .build();

        driveAvoidGateShootPos = follower.pathBuilder()
                .addPath(new BezierLine(avoidGatePose, shootPose))
                .setLinearHeadingInterpolation(avoidGatePose.getHeading(), shootPose.getHeading())
                .build();

        driveShootPosThirdRowIntakeSetUp = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, thirdRowIntakeSetUpPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), thirdRowIntakeSetUpPose.getHeading())
                .build();

        driveThirdRowIntakeSetUpIntakeThirdRow = follower.pathBuilder()
                .addPath(new BezierLine(thirdRowIntakeSetUpPose, intakeThirdRowPose))
                .setLinearHeadingInterpolation(thirdRowIntakeSetUpPose.getHeading(), intakeThirdRowPose.getHeading())
                .build();

        driveIntakeThirdRowShootPos = follower.pathBuilder()
                .addPath(new BezierLine(intakeThirdRowPose, shootPose))
                .setLinearHeadingInterpolation(intakeThirdRowPose.getHeading(), shootPose.getHeading())
                .build();

        driveShootPosGateSetUp = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, gateSetUpPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), gateSetUpPose.getHeading())
                .build();
    }

    public void statePathUpdate() {
        switch (pathState) {

            case DRIVE_STARTPOS_SHOOT_POS:
                if (!pathStarted) {
                    follower.followPath(driveStartPosShootPos, true);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    setPathState(PathState.SHOOT_PRELOAD_1);
                    pathStarted = false;
                }
                break;

            case SHOOT_PRELOAD_1:
                if (pathTimer.getElapsedTimeSeconds() > 0.85) {
                    MotorFeeder.setPower(-1.0);
                    helper.AutoTrack(0);
                }
                if (pathTimer.getElapsedTimeSeconds() > 3.85) {
                    MotorFeeder.setPower(1.0);
                    setPathState(PathState.DRIVE_SHOOT_POS_FIRSTROW_INTAKE_SETUP);
                }
                break;

            case DRIVE_SHOOT_POS_FIRSTROW_INTAKE_SETUP:
                if (!pathStarted) {
                    follower.followPath(driveShootPosFirstRowIntakeSetUpPos, true);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    setPathState(PathState.DRIVE_FIRSTROW_INTAKE_SETUP_INTAKE_FIRSTROW);
                }
                break;

            case DRIVE_FIRSTROW_INTAKE_SETUP_INTAKE_FIRSTROW:
                if (!pathStarted) {
                    follower.followPath(driveFirstRowIntakeSetUpPosIntakeFirstRowPos, true);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    setPathState(PathState.DRIVE_INTAKE_FIRSTROW_GATE_SETUP);
                }
                break;

            case DRIVE_INTAKE_FIRSTROW_GATE_SETUP:
                if (!pathStarted) {
                    follower.followPath(driveIntakeFirstRowPosGateSetUpPos, true);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    setPathState(PathState.DRIVE_INTAKE_GATE_SETUP_OPEN_GATE);
                }
                break;

            case DRIVE_INTAKE_GATE_SETUP_OPEN_GATE:
                if (!pathStarted) {
                    follower.followPath(driveGateSetUpPosOpenGatePos, true);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    setPathState(PathState.DRIVE_OPEN_GATE_SHOOT_POS);
                }
                break;

            case DRIVE_OPEN_GATE_SHOOT_POS:
                if (!pathStarted) {
                    follower.followPath(driveOpenGatePosShootPos, true);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    setPathState(PathState.SHOOT_2);
                }
                break;

            case SHOOT_2:
                if (pathTimer.getElapsedTimeSeconds() > 2.3) {
                    setPathState(PathState.DRIVE_SHOOT_POS_SECOND_ROW_INTAKE_SETUP);
                }
                break;

            case DRIVE_SHOOT_POS_SECOND_ROW_INTAKE_SETUP:
                if (!pathStarted) {
                    follower.followPath(driveShootPosSecondRowIntakeSetUp, true);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    setPathState(PathState.DRIVE_SECOND_ROW_INTAKE_SETUP_INTAKE_SECOND_ROW);
                }
                break;

            case DRIVE_SECOND_ROW_INTAKE_SETUP_INTAKE_SECOND_ROW:
                if (!pathStarted) {
                    follower.followPath(driveSecondRowIntakeSetUpIntakeSecondRow, true);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    setPathState(PathState.DRIVE_INTAKE_SECOND_ROW_AVOID_GATE);
                }
                break;

            case DRIVE_INTAKE_SECOND_ROW_AVOID_GATE:
                if (!pathStarted) {
                    follower.followPath(driveIntakeSecondRowAvoidGate, true);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    setPathState(PathState.DRIVE_AVOID_GATE_SHOOT_POS);
                }
                break;

            case DRIVE_AVOID_GATE_SHOOT_POS:
                if (!pathStarted) {
                    follower.followPath(driveAvoidGateShootPos, true);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    setPathState(PathState.SHOOT_3);
                }
                break;

            case SHOOT_3:
                if (pathTimer.getElapsedTimeSeconds() > 2.3) {
                    setPathState(PathState.DRIVE_SHOOT_POS_THIRD_ROW_INTAKE_SETUP);
                }
                break;

            case DRIVE_SHOOT_POS_THIRD_ROW_INTAKE_SETUP:
                if (!pathStarted) {
                    follower.followPath(driveShootPosThirdRowIntakeSetUp, true);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    setPathState(PathState.DRIVE_THIRD_ROW_INTAKE_SETUP_INTAKE_THIRD_ROW);
                }
                break;

            case DRIVE_THIRD_ROW_INTAKE_SETUP_INTAKE_THIRD_ROW:
                if (!pathStarted) {
                    follower.followPath(driveThirdRowIntakeSetUpIntakeThirdRow, true);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    setPathState(PathState.DRIVE_INTAKE_THIRD_ROW_SHOOT_POS);
                }
                break;

            case DRIVE_INTAKE_THIRD_ROW_SHOOT_POS:
                if (!pathStarted) {
                    follower.followPath(driveIntakeThirdRowShootPos, true);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    setPathState(PathState.SHOOT_4);
                }
                break;

            case SHOOT_4:
                if (pathTimer.getElapsedTimeSeconds() > 2.3) {
                    setPathState(PathState.DRIVE_SHOOT_POS_GATE_SETUP);
                }
                break;

            case DRIVE_SHOOT_POS_GATE_SETUP:
                if (!pathStarted) {
                    follower.followPath(driveShootPosGateSetUp, true);
                    pathStarted = true;
                }
                break;
        }
    }

    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
        pathStarted = false;
    }

    @Override
    public void init() {

        pathState = PathState.DRIVE_STARTPOS_SHOOT_POS;

        pathTimer = new Timer();
        OpModeTimer = new Timer();

        ServoConTurret = hardwareMap.get(Servo.class, "servo_con_turret");
        Turret = hardwareMap.get(DcMotorEx.class, "turret");
        MotorFeeder = hardwareMap.get(DcMotorEx.class, "motorizedtransfer");
        ServoConFront = hardwareMap.get(DcMotor.class, "servo_con_front_transfer");

        helper.init(hardwareMap, "Auto");

        follower = Constants.createFollower(hardwareMap);
        if (follower != null) {
            follower.setPose(startPose);
        }

        buildPaths();
    }

    @Override
    public void start() {

        PIDFCoefficients pidf = new PIDFCoefficients(520, 0, 5, 15.047);
        Turret.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidf);
        Turret.setVelocity(TurretVelocity);

        OpModeTimer.resetTimer();
        setPathState(pathState);
    }

    @Override
    public void loop() {

        helper.AutoIntake();

        switch (pathState) {
            case SHOOT_2:
            case SHOOT_3:
            case SHOOT_4:
                MotorFeeder.setPower(-1.0);
                if (helper.hasValidTarget()) {
                    helper.AutoTrack(1);
                } else {
                    ServoConTurret.setPosition(0.475);
                }
                break;

            default:
                MotorFeeder.setPower(1.0);
                break;
        }

        follower.update();
        statePathUpdate();
    }
}