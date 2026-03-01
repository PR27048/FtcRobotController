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

@Autonomous
public class Auto_BlueFar_WithPedroPath extends OpMode {
    private Follower follower;
    private Timer pathTimer, opModeTimer;
    PathState pathState;
    public enum PathState{
        SHOOT_PRELOAD,
        DRIVE_INTAKELOADINGZONE, // to pick up from loading zone

        DRIVE_INTAKEFIRSTSPIKE,
        NONE

    }

    private final Pose StartC1= new Pose(56,9.5, Math.toRadians(180)); //robot start position - near blue goal post

    private final Pose CollectLoadingZone= new Pose(10.6,9.5, Math.toRadians(180)); //robot start position - near blue goal post
    private final Pose CollectLoadingZoneBack = new Pose(18,9.5, Math.toRadians(180)); //robot start position - near blue goal post

    private final Pose CollectFirstSpike = new Pose(56,35.578856152513005, Math.toRadians(180)); //robot start position - near blue goal post


    private final Pose IntakeFullFirstSpike = new Pose(10.015597920277298,35.681109185441954, Math.toRadians(180)); //robot start position - near blue goal post

    private PathChain Intake_loadingzone, Intake_loadingzoneback,Intake_loadingzoneforward, Launch_Artifacts, Drive_firstspike;

    public void buildPaths(){
        //Blue Near position
        // Drive from A6 to C4 position for shooting initial loaded artifact
        Intake_loadingzone = follower.pathBuilder() //after shooting preload intake loading zone
                .addPath(new BezierLine(StartC1,CollectLoadingZone))
                .setLinearHeadingInterpolation(StartC1.getHeading(), CollectLoadingZone.getHeading())
                .build();
        Intake_loadingzoneback = follower.pathBuilder() //move back a bit
                .addPath(new BezierLine(CollectLoadingZone,CollectLoadingZoneBack))
                .setLinearHeadingInterpolation(CollectLoadingZone.getHeading(), CollectLoadingZoneBack.getHeading())
                .build();

        Intake_loadingzoneforward = follower.pathBuilder() //move forwards again
                .addPath(new BezierLine(CollectLoadingZoneBack,CollectLoadingZone))
                .setLinearHeadingInterpolation(CollectLoadingZoneBack.getHeading(), CollectLoadingZone.getHeading())
                .build();
        Launch_Artifacts = follower.pathBuilder() // move back to start to fire artifacts
                .addPath(new BezierLine(CollectLoadingZone,StartC1))
                .setLinearHeadingInterpolation(CollectLoadingZone.getHeading(), StartC1.getHeading())
                .build();
        Drive_firstspike = follower.pathBuilder()
                .addPath(new BezierLine(StartC1,CollectFirstSpike))
                .setLinearHeadingInterpolation(StartC1.getHeading(), CollectFirstSpike.getHeading())
                .build();
        /*Intake_FirstSpike = follower.pathBuilder()
                .addPath(new BezierLine(CollectFirstSpike,IntakeFullFirstSpike))
                .setLinearHeadingInterpolation(CollectFirstSpike.getHeading(), IntakeFullFirstSpike.getHeading())
                .build();*/

    }

    public void statePathUpdate(){
        switch (pathState){
            case SHOOT_PRELOAD:
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 5){
                    setPathState(PathState.DRIVE_INTAKELOADINGZONE);
                    telemetry.addLine("Done Preload Path and Shoot ");
                }

                break;
            case DRIVE_INTAKELOADINGZONE:
                follower.followPath(Intake_loadingzone,true);
               // if(!follower.isBusy()) {
                    follower.followPath(Intake_loadingzoneback, true);
                //}
                //if(!follower.isBusy()) {
                    follower.followPath(Intake_loadingzoneforward, true);
                //}
                //if(!follower.isBusy())
                follower.followPath(Launch_Artifacts,true);
                setPathState(PathState.DRIVE_INTAKEFIRSTSPIKE);

                break;

            case DRIVE_INTAKEFIRSTSPIKE:

                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 5){
                   // follower.followPath(Intake_FirstSpike);
                    setPathState(PathState.DRIVE_INTAKEFIRSTSPIKE);
                    telemetry.addLine("Done Preload Path and Shoot ");
                }
            default:
                telemetry.addLine("All done");
                break;

        }
    }

    public void setPathState(PathState newState){
        pathState = newState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        pathState = PathState.SHOOT_PRELOAD;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        opModeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);
        //TODO add in any other init mechanics
        // serviceHelper.init(hardwareMap);
        buildPaths();
        follower.setPose(StartC1);
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
