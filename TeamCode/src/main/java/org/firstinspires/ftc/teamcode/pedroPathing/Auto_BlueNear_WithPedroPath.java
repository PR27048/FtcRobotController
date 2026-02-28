package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.decode.DecodeAuto;
import org.firstinspires.ftc.teamcode.decode.ServiceHelper;

@Autonomous
public class Auto_BlueNear_WithPedroPath extends OpMode {
    private Follower follower;
    private Timer pathTimer, opModeTimer;
    PathState pathState;
    public enum PathState{
        DRIVE_PRELOAD_STARTPOS_SHOOTPOS, //drive the preloaded robot to shoot position
        SHOOT_PRELOAD, // to shoot preloaded artifacts
        DRIVE_B4_INTAKE_POS, // drive and intake the artifacts from B4 line
        SHOOT_B4_ARTIFACT, //shoot the artifacts we intake from B4 line
        DRIVE_B3_INTAKE_POS, //drive and intake the artifacts from B3 line
        SHOOT_B3_ARTIFACT, //shoot the artifacts we intake from B3 line
        DRIVE_B2_INTAKE_POS, //drive and intake the artifacts from B2 line
        SHOOT_B2_ARTIFACT, //shoot the artifacts we intake from B2 line
        DRIVE_ENDPOS //Move the robot from launching line to get leave point
    }

    private final Pose bluA6StartPos= new Pose(21.809358752166382,122.80069324090121, Math.toRadians(144)); //robot start position - near blue goal post
    private final Pose bluC4NearShootPos = new Pose(56.5,86.2, Math.toRadians(138)); //robot near goal shoot position - near blue goal post
    private final Pose bluIntkC4AlignPos = new Pose(56.250433275563275,83.45476603119583, Math.toRadians(180));
    private final Pose bluIntkB4Pos = new Pose(18.71403812824958,83.50433275563257, Math.toRadians(180));
    private final Pose bluIntkC3AlignPos = new Pose(55.751299826689774,58.747660311958384, Math.toRadians(180));
    private final Pose bluIntkB3Pos = new Pose(17.629116117850952,58.9, Math.toRadians(180));

    private PathChain bluA6PreloadStartPosShootPos, bluIntkC4AlignPosPath, bluIntkB4PosPath, bluShootB4IntkArt, bluIntkC3AlignPosPath, bluIntkB3PosPath, bluShootB3IntkArt;

    public void buildPaths(){
        //Blue Near position
        // Drive from A6 to C4 position for shooting initial loaded artifact
        bluA6PreloadStartPosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(bluA6StartPos,bluC4NearShootPos))
                .setLinearHeadingInterpolation(bluA6StartPos.getHeading(), bluC4NearShootPos.getHeading())
                .build();
        //Align the robot after preload shooting so its aligned to intake the artifacts on B4 line
        bluIntkC4AlignPosPath = follower.pathBuilder()
                .addPath(new BezierLine(bluC4NearShootPos,bluIntkC4AlignPos))
                .setLinearHeadingInterpolation(bluC4NearShootPos.getHeading(), bluIntkC4AlignPos.getHeading())
                .build();
        //Drive to B4 path to intake the artifacts pre-placed on the B4 line
        bluIntkB4PosPath = follower.pathBuilder()
                .addPath(new BezierLine(bluIntkC4AlignPos,bluIntkB4Pos))
                .setLinearHeadingInterpolation(bluIntkC4AlignPos.getHeading(), bluIntkB4Pos.getHeading())
                .build();

        //Drive to near goal shoot position and shoot B4 artifact after intake
        bluShootB4IntkArt = follower.pathBuilder()
                .addPath(new BezierLine(bluIntkB4Pos, bluC4NearShootPos))
                .setLinearHeadingInterpolation(bluIntkB4Pos.getHeading(), bluC4NearShootPos.getHeading())
                .build();

        //Align the robot after C4 line shooting so its aligned to intake the artifacts on B3 line
        bluIntkC3AlignPosPath = follower.pathBuilder()
                .addPath(new BezierLine(bluC4NearShootPos,bluIntkC3AlignPos))
                .setLinearHeadingInterpolation(bluC4NearShootPos.getHeading(), bluIntkC4AlignPos.getHeading())
                .build();

        //Drive to B3 path to intake the artifacts pre-placed on the B3 line
        bluIntkB3PosPath = follower.pathBuilder()
                .addPath(new BezierLine(bluIntkC3AlignPos,bluIntkB3Pos))
                .setLinearHeadingInterpolation(bluIntkC3AlignPos.getHeading(), bluIntkB3Pos.getHeading())
                .build();

        //Drive to near goal shoot position and shoot B3 artifact after intake
        bluShootB3IntkArt = follower.pathBuilder()
                .addPath(new BezierLine(bluIntkB3Pos, bluC4NearShootPos))
                .setLinearHeadingInterpolation(bluIntkB3Pos.getHeading(), bluC4NearShootPos.getHeading())
                .build();
    }

    public void statePathUpdate(){
        switch (pathState){
            case DRIVE_PRELOAD_STARTPOS_SHOOTPOS:
                follower.followPath(bluA6PreloadStartPosShootPos,true);
                setPathState(PathState.SHOOT_PRELOAD); //reset timer and make new state
                break;
            case SHOOT_PRELOAD:
                //TODO Add logic for flywheel to shoot artifacts
                //serviceHelper.AutoLaunch();
                //check if follower has done its path & and check that 5 seconds has elapsed
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 5){
                    follower.followPath(bluIntkC4AlignPosPath, true);
                    follower.followPath(bluIntkB4PosPath, true);
                    setPathState(PathState.DRIVE_B4_INTAKE_POS);
                    telemetry.addLine("Done Preload Path and Shoot ");
                }
                break;
            case DRIVE_B4_INTAKE_POS:
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 3){
                    follower.followPath(bluShootB4IntkArt, true);
                    setPathState(PathState.SHOOT_B4_ARTIFACT);
                    telemetry.addLine("Done Intake of Artifact from B4 path");
                }
                break;
            case SHOOT_B4_ARTIFACT:
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 5){
                    follower.followPath(bluIntkC3AlignPosPath, true);
                    follower.followPath(bluIntkB3PosPath, true);
                    setPathState(PathState.DRIVE_B3_INTAKE_POS);
                    telemetry.addLine("Done shooting Artifact from B4 path line");
                }
                break;
            case DRIVE_B3_INTAKE_POS:
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 3){
                    follower.followPath(bluShootB3IntkArt, true);//
                    setPathState(PathState.SHOOT_B3_ARTIFACT);
                    telemetry.addLine("Done Intake of Artifact from B4 path");
                }
                break;
            case SHOOT_B3_ARTIFACT:
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 5){

                    //setPathState(PathState.);
                    telemetry.addLine("Done shooting Artifact from B3 path line");
                }
                break;
            default:
                telemetry.addLine("No state commanded");
                break;
        }
    }

    public void setPathState(PathState newState){
        pathState = newState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        pathState = PathState.DRIVE_PRELOAD_STARTPOS_SHOOTPOS;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        opModeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);
        //TODO add in any other init mechanics
        // serviceHelper.init(hardwareMap);
        buildPaths();
        follower.setPose(bluA6StartPos);
    }


    public void start(){
        opModeTimer.resetTimer();
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
