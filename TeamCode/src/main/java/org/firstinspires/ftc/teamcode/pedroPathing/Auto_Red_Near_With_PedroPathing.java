package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.pedropathing.util.Timer;
@Disabled
public class Auto_Red_Near_With_PedroPathing extends OpMode {

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

         SHOOT_PRELOAD_2,

        DRIVE_SHOOT_POS_SECOND_ROW_INTAKE_SETUP,

        DRIVE_SECOND_ROW_INTAKE_SETUP_INTAKE_SECOND_ROW,

        DRIVE_INTAKE_SECOND_ROW_AVOID_GATE,

        DRIVE_AVOID_GATE_SHOOT_POS,

         SHOOT_PRELOAD_3,

        DRIVE_SHOOT_POS_THIRD_ROW_INTAKE_SETUP,

        DRIVE_THIRD_ROW_INTAKE_SETUP_INTAKE_THIRD_ROW,

        DRIVE_INTAKE_THIRD_ROW_SHOOT_POS,

         SHOOT_PRELOAD_4,

        DRIVE_SHOOT_POS_GATE_SETUP
    }

    PathState pathState;

    private final Pose startPose = new Pose(123.28729281767956, 122.56353591160222, Math.toRadians(37));
    private final Pose shootPose = new Pose(91.80110497237568, 89.75966850828729, Math.toRadians(47));
    private final Pose firstRowIntakeSetUpPose = new Pose(92.12154696132599, 83.19337016574585, Math.toRadians(0));
    private final Pose intakeFirstRowPose = new Pose(129.38674033149172, 83.13259668508286, Math.toRadians(0));
    private final Pose gateSetUpPose = new Pose(119.20994475138122, 71.01657458563535, Math.toRadians(0));
    private final Pose openGatePose = new Pose(128.30939226519337, 70.69060773480662, Math.toRadians(0));
    // private final Pose shootPose = new Pose(91.80110497237568, 89.75966850828729, Math.toRadians(47));
    private final Pose  secondRowIntakeSetUpPose = new Pose(95.3259668508287, 59.7900552486188, Math.toRadians(0));
    private final Pose intakeSecondRowPose = new Pose(134.98342541436463, 58.68508287292817, Math.toRadians(0));
    private final Pose avoidGatePose = new Pose(119.01104972375691, 58.961325966850815, Math.toRadians(0));
    // private final Pose shootPose = new Pose(91.80110497237568, 89.75966850828729, Math.toRadians(47));
    private final Pose thirdRowIntakeSetUpPose = new Pose(96.62430939226519, 35.91712707182322, Math.toRadians(0));
    private final Pose intakeThirdRowPose = new Pose(135.41988950276243, 35.75690607734807, Math.toRadians(0));
    // private final Pose shootPose = new Pose(91.80110497237568, 89.75966850828729, Math.toRadians(47));
    private PathChain driveStartPosShootPos, driveShootPosFirstRowIntakeSetUpPos, driveFirstRowIntakeSetUpPosIntakeFirstRowPos, driveIntakeFirstRowPosGateSetUpPos, driveGateSetUpPosOpenGatePos, driveOpenGatePosShootPos, driveShootPosSecondRowIntakeSetUp, driveSecondRowIntakeSetUpIntakeSecondRow, driveIntakeSecondRowAvoidGate, driveAvoidGateShootPos, driveShootPosThirdRowIntakeSetUp, driveThirdRowIntakeSetUpIntakeThirdRow, driveIntakeThirdRowShootPos, driveShootPosGateSetUp;
    // private final Pose gateSetUpPose = new Pose(119.20994475138122, 71.01657458563535, Math.toRadians(0));
    public void buildPaths() {
        // put in coordinates for starting pose > ending pose
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
                follower.followPath(driveStartPosShootPos, true);
                setPathState(PathState.SHOOT_PRELOAD_1); // reset the timer and make new state
                telemetry.addLine("done path 1");
                break;
            case SHOOT_PRELOAD_1:
                // check if follower done its path
                if (pathTimer.getElapsedTimeSeconds() > 4) {
                    // TODO ADD LOGIC TO FLYWHEEL SHOOTER
                    setPathState(PathState.DRIVE_SHOOT_POS_FIRSTROW_INTAKE_SETUP);
                }
                break;
            case DRIVE_SHOOT_POS_FIRSTROW_INTAKE_SETUP:
                if (!follower.isBusy()) {
                    follower.followPath(driveShootPosFirstRowIntakeSetUpPos, true);
                    setPathState(PathState.DRIVE_FIRSTROW_INTAKE_SETUP_INTAKE_FIRSTROW);
                    telemetry.addLine("done path 2");
                }
                break;
            case DRIVE_FIRSTROW_INTAKE_SETUP_INTAKE_FIRSTROW:
                if (!follower.isBusy()) {
                    follower.followPath(driveFirstRowIntakeSetUpPosIntakeFirstRowPos, true);
                    setPathState(PathState.DRIVE_INTAKE_FIRSTROW_GATE_SETUP);
                    telemetry.addLine("done path 3");
                }
                break;
            case DRIVE_INTAKE_FIRSTROW_GATE_SETUP:
                if (!follower.isBusy()) {
                    follower.followPath(driveIntakeFirstRowPosGateSetUpPos, true);
                    setPathState(PathState.DRIVE_INTAKE_GATE_SETUP_OPEN_GATE);
                    telemetry.addLine("done path 4");
                }
                break;
            case DRIVE_INTAKE_GATE_SETUP_OPEN_GATE:
                if (!follower.isBusy()) {
                    follower.followPath(driveGateSetUpPosOpenGatePos, true);
                    setPathState(PathState.DRIVE_OPEN_GATE_SHOOT_POS);
                    telemetry.addLine("done path 5");
                }
                break;
            case DRIVE_OPEN_GATE_SHOOT_POS:
                if (!follower.isBusy()) {
                    follower.followPath(driveOpenGatePosShootPos, true);
                    setPathState(PathState.SHOOT_PRELOAD_2);
                    telemetry.addLine("done path 6");
                }
                break;
            case SHOOT_PRELOAD_2:
                if (pathTimer.getElapsedTimeSeconds() > 4) {
                    // TODO ADD LOGIC TO FLYWHEEL SHOOTER
                    setPathState(PathState.DRIVE_SHOOT_POS_SECOND_ROW_INTAKE_SETUP);
                }

                break;
            case DRIVE_SHOOT_POS_SECOND_ROW_INTAKE_SETUP:
                if (!follower.isBusy()) {
                    follower.followPath(driveShootPosSecondRowIntakeSetUp, true);
                    setPathState(PathState.DRIVE_SECOND_ROW_INTAKE_SETUP_INTAKE_SECOND_ROW);
                    telemetry.addLine("done path 7");
                }
                break;
            case DRIVE_SECOND_ROW_INTAKE_SETUP_INTAKE_SECOND_ROW:
                if (!follower.isBusy()) {
                    follower.followPath(driveSecondRowIntakeSetUpIntakeSecondRow, true);
                    setPathState(PathState.DRIVE_INTAKE_SECOND_ROW_AVOID_GATE);
                    telemetry.addLine("done path 8");
                }
                break;
            case DRIVE_INTAKE_SECOND_ROW_AVOID_GATE:
                if (!follower.isBusy()) {
                    follower.followPath(driveIntakeSecondRowAvoidGate, true);
                    setPathState(PathState.DRIVE_AVOID_GATE_SHOOT_POS);
                    telemetry.addLine("done path 9");
                }
                break;
            case DRIVE_AVOID_GATE_SHOOT_POS:
                if (!follower.isBusy()) {
                    follower.followPath(driveAvoidGateShootPos, true);
                    setPathState(PathState.SHOOT_PRELOAD_3);
                    telemetry.addLine("done path 10");
                }
                break;
            case SHOOT_PRELOAD_3:
                if (pathTimer.getElapsedTimeSeconds() > 4) {
                    // TODO ADD LOGIC TO FLYWHEEL SHOOTER
                    setPathState(PathState.DRIVE_SHOOT_POS_THIRD_ROW_INTAKE_SETUP);
                }
                break;
            case DRIVE_SHOOT_POS_THIRD_ROW_INTAKE_SETUP:
                if (!follower.isBusy()) {
                    follower.followPath(driveShootPosThirdRowIntakeSetUp, true);
                    setPathState(PathState.DRIVE_THIRD_ROW_INTAKE_SETUP_INTAKE_THIRD_ROW);
                    telemetry.addLine("done path 11");
                }
                break;
            case DRIVE_THIRD_ROW_INTAKE_SETUP_INTAKE_THIRD_ROW:
                if (!follower.isBusy()) {
                    follower.followPath(driveThirdRowIntakeSetUpIntakeThirdRow, true);
                    setPathState(PathState.DRIVE_INTAKE_THIRD_ROW_SHOOT_POS);
                    telemetry.addLine("done path 12");
                }
                break;
            case DRIVE_INTAKE_THIRD_ROW_SHOOT_POS:
                if (!follower.isBusy()) {
                    follower.followPath(driveIntakeThirdRowShootPos, true);
                    setPathState(PathState.SHOOT_PRELOAD_4);
                    telemetry.addLine("done path 13");
                }
                break;
            case SHOOT_PRELOAD_4:
                if (pathTimer.getElapsedTimeSeconds() > 4) {
                    // TODO ADD LOGIC TO FLYWHEEL SHOOTER
                    setPathState(PathState.DRIVE_SHOOT_POS_GATE_SETUP);
                }
                break;
            case DRIVE_SHOOT_POS_GATE_SETUP:
                if (!follower.isBusy())
                    follower.followPath(driveShootPosGateSetUp, true);
                    telemetry.addLine("done path 14");
                break;
            default:
                telemetry.addLine("no state commanded");
                break;
        }
    }

    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        pathState = PathState.DRIVE_STARTPOS_SHOOT_POS;
        pathTimer = new Timer();
        OpModeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        // TODO add in any other init mechanisms

        buildPaths();
        follower.setPose(startPose);
    }

    public void start() {
        OpModeTimer.resetTimer();
        setPathState(pathState);
    }

    @Override
    public void loop() {
        follower.update();
        statePathUpdate();


        telemetry.addData("path state", pathState.toString());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("Path time", pathTimer.getElapsedTimeSeconds());
    }
}
