package org.firstinspires.ftc.teamcode.Tests;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.decode.ServiceHelper;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Disabled
@TeleOp
public class AutoWithPedroPath extends OpMode {
    ServiceHelper serviceHelper = new ServiceHelper();
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
        DRIVE_FIRST_ART_POS,
        SHOOT_FIRST_ART_POS,
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

    private Pose startPos, shootPos, grabfirstArtPoseAngle, grabfirstArtPose, shootfirstArtPose, endPose;
    private Pose bluC1StartPos, bluC4ShootPos, bluIntkC4AlignPos, bluIntkC4Pos, bluEndPos;
    /*private final Pose startPos= new Pose(56.000,8.000, Math.toRadians(90));
    private final Pose  shootPos = new Pose(56,86.91161178509532, Math.toRadians(133));
    private final Pose grabfirstArtPose = new Pose(19.424610051993064,84.80069324090124, Math.toRadians(180));
    private final Pose shootfirstArtPose = new Pose(55.999999999999986,87.41074523396881, Math.toRadians(180));
    private final Pose endPose = new Pose(56,34.00346620450607, Math.toRadians(180));
*/

    // driveStartPosShootPos -> drive from initial position to shoot position for shooting preloaded artifact
    // driveShootPosEndPos -> at the end when all artifacts are done move away from shooting line to get the leave point
    private PathChain driveStartPosShootPos, grabfirstArtPoseAnglePath, driveGrabfirstArtPose,shootGrabfirstArtPose, driveShootPosEndPos;
    private PathChain bluC4ShootPosPath, bluIntkC4AlignPosPath, bluIntkC4PosPath, bluIntkC4ShootPosPath, bluEndPosPath;
    public void buildPaths(){
        //Blue Far position
        // Drive from C1 to C4 for initial loaded artifact to get to shoot position
        bluC4ShootPosPath = follower.pathBuilder()
                .addPath(new BezierLine(bluC1StartPos,bluC4ShootPos))
                .setLinearHeadingInterpolation(bluC1StartPos.getHeading(), bluC4ShootPos.getHeading())
                .build();
        bluIntkC4AlignPosPath = follower.pathBuilder()
                .addPath(new BezierLine(bluC4ShootPos,bluIntkC4AlignPos))
                .setLinearHeadingInterpolation(bluC4ShootPos.getHeading(), bluIntkC4AlignPos.getHeading())
                .build();
        bluIntkC4PosPath = follower.pathBuilder()
                .addPath(new BezierLine(bluIntkC4AlignPos,bluIntkC4Pos))
                .setLinearHeadingInterpolation(bluIntkC4AlignPos.getHeading(), bluIntkC4Pos.getHeading())
                .build();

        //coordinates for driving to shoot position and shooting first artifact line which were grabbed
        shootGrabfirstArtPose = follower.pathBuilder()
                .addPath(new BezierLine(grabfirstArtPose, shootfirstArtPose))
                .setLinearHeadingInterpolation(grabfirstArtPose.getHeading(), shootfirstArtPose.getHeading())
                .build();
        //put in the coordinates for final shoot position to end position which is not on the line to get leave point
        driveShootPosEndPos = follower.pathBuilder()
                .addPath(new BezierLine(shootfirstArtPose,shootPos))
                .setLinearHeadingInterpolation(shootfirstArtPose.getHeading(), shootPos.getHeading())
                .build();

       /* //put in the coordinates for initial start to shoot position
        driveStartPosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPos,shootPos))
                .setLinearHeadingInterpolation(startPos.getHeading(), shootPos.getHeading())
                .build();
        grabfirstArtPoseAnglePath = follower.pathBuilder()
                .addPath(new BezierLine(shootPos,grabfirstArtPoseAngle))
                .setLinearHeadingInterpolation(shootPos.getHeading(), grabfirstArtPoseAngle.getHeading())
                .build();
        //coordinates for driving and grabbing towards first artifact line
        driveGrabfirstArtPose = follower.pathBuilder()
                .addPath(new BezierLine(shootPos, grabfirstArtPose))
                .setLinearHeadingInterpolation(shootPos.getHeading(), grabfirstArtPose.getHeading())
                .build();
        //coordinates for driving to shoot position and shooting first artifact line which were grabbed
        shootGrabfirstArtPose = follower.pathBuilder()
                .addPath(new BezierLine(grabfirstArtPose, shootfirstArtPose))
                .setLinearHeadingInterpolation(grabfirstArtPose.getHeading(), shootfirstArtPose.getHeading())
                .build();
        //put in the coordinates for final shoot position to end position which is not on the line to get leave point
        driveShootPosEndPos = follower.pathBuilder()
                .addPath(new BezierLine(shootfirstArtPose,shootPos))
                .setLinearHeadingInterpolation(shootfirstArtPose.getHeading(), shootPos.getHeading())
                .build();*/
    }

    public void statePathUpdate(){
        switch (pathState){
            case DRIVE_STARTPOS_SHOOT_POS:
                follower.followPath(driveStartPosShootPos,true);
                setPathState(PathState.SHOOT_PRELOAD); //reset timer and make new state
                break;
            case SHOOT_PRELOAD:
                //TODO Add logic for flywheel to shoot artifacts
                //serviceHelper.AutoLaunch();
                //check if follower has done its path & and check that 5 seconds has elapsed
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 5){
                    follower.followPath(grabfirstArtPoseAnglePath, true);
                    follower.followPath(driveGrabfirstArtPose, true);
                    setPathState(PathState.DRIVE_FIRST_ART_POS);
                    telemetry.addLine("Done Path 1");
                }
                break;
            case DRIVE_FIRST_ART_POS:
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 3){
                    follower.followPath(shootGrabfirstArtPose, true);
                    setPathState(PathState.SHOOT_FIRST_ART_POS);
                    telemetry.addLine("Done grabbing path 1");
                }
                break;
            case SHOOT_FIRST_ART_POS:
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 5){
                    follower.followPath(driveShootPosEndPos, true);
                    setPathState(PathState.DRIVE_SHOOT_ENDPOS);
                    telemetry.addLine("Done Path 2");
                }
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

    }

    @Override
    public void init_loop() {
        if (gamepad1.dpad_right && !lastRight) {
            selMode = AutoMode.BLUE_FAR;
            runBlueFar();
            buildPaths();
            follower.setPose(startPos);
        }

        if (gamepad1.dpad_left && !lastLeft) {
            selMode = AutoMode.BLUE_NEAR;
            runBlueClose();
            buildPaths();
            follower.setPose(startPos);
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


    }

    public void start(){
        opModeTimer.resetTimer();
        setPathState(pathState);
    }

    @Override
    public void loop() {
        selMode = AutoMode.NONE;
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
        startPos= new Pose(56.000,8.000, Math.toRadians(90));
        shootPos = new Pose(56,83.41767764298092, Math.toRadians(135));
        grabfirstArtPoseAngle = new Pose(56,82.91854419410745, Math.toRadians(180));
        grabfirstArtPose = new Pose(22.045060658578855,82.77469670710575, Math.toRadians(180));
        endPose = new Pose(56,34.00346620450607, Math.toRadians(180));


        bluC1StartPos= new Pose(56.000,8.000, Math.toRadians(90));
        bluC4ShootPos = new Pose(56,83.41767764298092, Math.toRadians(135));
        bluIntkC4AlignPos = new Pose(56,82.91854419410745, Math.toRadians(180));
        bluIntkC4Pos = new Pose(22.045060658578855,82.77469670710575, Math.toRadians(180));
        bluEndPos = new Pose(56,34.00346620450607, Math.toRadians(180));

    }

}
