package org.firstinspires.ftc.teamcode.Tests;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Disabled
@TeleOp
public class SampleAutoPathing extends OpMode {
    private Follower follower;
    private Timer pathTimer, opModeTimer;
    boolean lastLeft = false;
    boolean lastRight = false;
    boolean lastUp = false;
    boolean lastDown = false;
    boolean lastGuide = false;
    public enum PathState{
        //START POSITION_END POSITION
        //DRIVE > MOVEMENT STATE
        //SHOOT > ATTEMPT TO SCORE THE ARTIFACT
        DRIVE_STARTPOS_SHOOT_POS,
        SHOOT_PRELOAD, // to shoot preloaded artifacts
        DRIVE_SHOOT_ENDPOS
    }

    enum AutoMode {
        NONE,
        BLUE_NEAR,
        BLUE_FAR,
        RED_NEAR,
        RED_FAR
    }

    AutoMode selMode = AutoMode.NONE;

    PathState pathState;

    //private final Pose startPos= new Pose(56.000,8.000, Math.toRadians(90));
    //private final Pose  shootPos = new Pose(56,34.00346620450607, Math.toRadians(180));
    //private final Pose endPose = new Pose(56,34.00346620450607, Math.toRadians(180));

    private final Pose startPos=  new Pose(20.386209877877445,122.39783853885227, Math.toRadians(138)); //TODO update values
    private final Pose  shootPos = new Pose(46.415043769588245,96.90020533880903, Math.toRadians(138)); //TODO update values
    private final Pose endPose = new Pose(63.76759969739543,105.75355019993515, Math.toRadians(98)); //TODO update values

    // driveStartPosShootPos -> drive from initial position to shoot position for shooting preloaded artifact
    // driveShootPosEndPos -> at the end when all artifacts are done move away from shooting line to get the leave point
    private PathChain driveStartPosShootPos, driveShootPosEndPos;

    public void buildPaths(){
        //put in the coordinates for initial start to shoot position
        driveStartPosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPos,shootPos))
                .setLinearHeadingInterpolation(startPos.getHeading(), shootPos.getHeading())
                .build();
        //put in the coordinates for final shoot position to end position which is not on the line to get leave point
        driveShootPosEndPos = follower.pathBuilder()
                .addPath(new BezierLine(shootPos,endPose))
                .setLinearHeadingInterpolation(shootPos.getHeading(), endPose.getHeading())
                .build();
    }

    public void statePathUpdate(){
        telemetry.addLine("pathState" + pathState);
        switch (pathState){

            case DRIVE_STARTPOS_SHOOT_POS:
                telemetry.addLine("driveStartPosShootPos" );
                follower.followPath(driveStartPosShootPos,true);
                setPathState(PathState.SHOOT_PRELOAD); //reset timer and make new state
                telemetry.addLine("driveStartPosShootPos done" );
                break;
            case SHOOT_PRELOAD:
                //TODO Add logic for flywheel to shoot artifacts

                //check if follower has done its path & and check that 5 seconds has elapsed
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 5){
                   follower.followPath(driveShootPosEndPos, true);
                    setPathState(PathState.DRIVE_SHOOT_ENDPOS);
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
       // serviceHelper.init(hardwareMap);
        buildPaths();
        follower.setPose(startPos);

    }

    /*
    @Override
    public void init_loop() {
        if (gamepad1.dpad_right && !lastRight) {
            selMode = AutoMode.BLUE_FAR;
            runBlueFar();
        }

        if (gamepad1.dpad_left && !lastLeft) {
            selMode = AutoMode.BLUE_NEAR;
            runBlueClose();
        }
        if (gamepad1.dpad_up && !lastUp) {
            selMode = AutoMode.RED_FAR;
        }
        if (gamepad1.dpad_down && !lastDown) {
            selMode = AutoMode.RED_NEAR;
        }
        telemetry.addLine("Select Auto");
        telemetry.addLine("D-Pad to select auto \n right:BLUE_FAR-C1 \n left:BLUE_NEAR-A5 \n up:RED_FAR-D1 \n down:RED_NEAR-E5");
        telemetry.addData("\n Selected Auto", selMode);
        telemetry.addLine("Press START when ready");
        telemetry.update();

        lastLeft = gamepad1.dpad_left;
        lastRight = gamepad1.dpad_right;
        lastUp = gamepad1.dpad_up;
        lastDown = gamepad1.dpad_down;
        lastGuide = gamepad1.guide;
    }*/

    public void start(){
        opModeTimer.resetTimer();
        setPathState(pathState);
    }

    @Override
    public void loop() {
       // selMode = AutoMode.NONE;
        follower.update();
        statePathUpdate();
        telemetry.addData("path state", pathState.toString());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("Path time", pathTimer.getElapsedTimeSeconds());
    }

    private void runBlueClose()
    {
        /*
        startPos = new Pose(20.386209877877445,122.39783853885227, Math.toRadians(138)); //TODO update values
        shootPos = new Pose(46.415043769588245,96.90020533880903, Math.toRadians(138)); //TODO update values
        endPose = new Pose(63.76759969739543,105.75355019993515, Math.toRadians(98)); //TODO update values
        serviceHelper.SetTurretVelocity();*/
    }
    private void runBlueFar()
    {
       /* startPos = new Pose(56.000,8.000, Math.toRadians(90));
        shootPos = new Pose(56,34.00346620450607, Math.toRadians(180));
        endPose = new Pose(56,34.00346620450607, Math.toRadians(180)); //TODO update this
       serviceHelper.SetTurretVelocity(); */
    }

}
