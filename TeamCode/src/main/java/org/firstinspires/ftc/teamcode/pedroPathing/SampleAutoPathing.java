package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class SampleAutoPathing extends OpMode {
    private Follower follower;
    private Timer pathTimer, opModeTimer;

    public enum PathState{
        //START POSITION_END POSITION
        //DRIVE > MOVEMENT STATE
        //SHOOT > ATTEMPT TO SCORE THE ARTIFACT
        DRIVE_STARTPOS_SHOOT_POS,
        SHOOT_PRELOAD,
        DRIVE_SHOOT_ENDPOS
    }

    PathState pathState;

    private final Pose startPos = new Pose(20.386209877877445,122.39783853885227, Math.toRadians(138)); //TODO update values
    private final Pose shootPos = new Pose(46.415043769588245,96.90020533880903, Math.toRadians(138)); //TODO update values
    private final Pose endPose = new Pose(63.76759969739543,105.75355019993515, Math.toRadians(98)); //TODO update values
    private PathChain driveStartPosShootPos, driveShootPosEndPos;

    public void buildPaths(){
        //put in the coordinates for starting pose > ending pose
        driveStartPosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPos,shootPos))
                .setLinearHeadingInterpolation(startPos.getHeading(), shootPos.getHeading())
                .build();
        driveShootPosEndPos = follower.pathBuilder()
                .addPath(new BezierLine(shootPos,endPose))
                .setLinearHeadingInterpolation(shootPos.getHeading(), endPose.getHeading())
                .build();
    }

    public void statePathUpdate(){
        switch (pathState){
            case DRIVE_STARTPOS_SHOOT_POS:
                follower.followPath(driveStartPosShootPos,true);
                pathState = PathState.SHOOT_PRELOAD;
                break;
            case SHOOT_PRELOAD:
                //TODO add logic to flywheel shooter
                //check if follower has done its path
                // and check that 5 seconds has elapsed
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 5){
                    follower.followPath(driveShootPosEndPos, true);
                    setPathState(PathState.DRIVE_SHOOT_ENDPOS);
                    //TODO add logic to flywheel shooter
                    telemetry.addLine("Done Path 1");

                }
                break;
            case DRIVE_SHOOT_ENDPOS:
                //all done
                if(!follower.isBusy()){
                    telemetry.addLine("Done all Paths");
                }
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
        pathState = PathState.DRIVE_STARTPOS_SHOOT_POS;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        opModeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);
        //TODO add in any other init mechanics

        buildPaths();
        follower.setPose(startPos);
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
