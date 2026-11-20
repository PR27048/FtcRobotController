package org.firstinspires.ftc.teamcode.Autonoumous;


import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.Teleops.Utilities;
@Autonomous
public class AutoThreeClose extends OpMode {

    private final Utilities utils = new Utilities();

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private Follower follower;

    public int pathState;

    private final Pose start = poseFactory.of(56, 8, 270);
    private final Pose shoot = poseFactory.of(56, 20, 270);
    private final Pose prePickUp = poseFactory.of(9, 20, 270);
    private final Pose pickUp = poseFactory.of(9, 8.5, 270);
    private final Pose flower = poseFactory.of(12.5126, 46.9079, 180);
    private final Pose prePark = poseFactory.of(9.8822, 71.1769, 270);
    private final Pose park = poseFactory.of(10.1353, 94.6676, 270);
    public Path shootFirst() {
        return Paths.line(start, shoot).linear(start, shoot);
    }

    public Path prePickUpPollen() {
        return Paths.line(shoot, prePickUp).linear(shoot, prePickUp);
    }

    public Path pickUpPollen() {
        return Paths.line(prePickUp, pickUp).linear(prePickUp, pickUp);
    }

    public Path GoBackShootAgain() {
        return Paths.line(pickUp, shoot).linear(pickUp, shoot);
    }

    public Path getFlower() {
        return Paths.line(shoot, flower).linear(shoot, flower);
    }

    public Path shootAgainAgain() {
        return Paths.line(flower, shoot).linear(flower, shoot);
    }

    public Path preParkIt() {
        return Paths.line(shoot, prePark).linear(shoot, prePark);
    }

    public Path park() {
        return Paths.line(prePark, park).linear(prePark, park);
    }

    public void autoPathUpdate(){
        switch(pathState){
            case 1:
                follower.follow(shootFirst());
             //   utils.SetShooterVelocity(.7);
                // utils.Setintake(.7);
                pathState=2;
                break;
            case 2:
                if (follower.isBusy()) {
//                    utils.SetGate(1);

                }
                pathState = 3;
                break;
            case 3:
//                utils.SetGate(0);
                follower.follow(prePickUpPollen());
                pathState = 4;
                break;
            case 4:
                follower.follow(pickUpPollen());
                pathState = 5;
                break;
            case 5:
                follower.follow(GoBackShootAgain());
                pathState = 6;
                break;
            case 6:
                if (follower.isBusy()) {
                    //  utils.SetGate(1);
                }
                pathState = 7;

                break;
            case 7:
//                utils.SetGate(0);
                follower.follow(getFlower());
                pathState = 8;
                break;
            case 8:
                if(follower.isBusy()) {
                    //   utils.SetGate(0);
                }
                pathState = 9;

                break;
           /* case 9:
                follower.follow(shootAgainAgain());
                pathState = 9;
                break;
            case 10:
//                utils.SetGate(1);
                pathState = 10;
                break;
            case 11:
//                utils.SetGate(0);
                follower.follow(preParkIt());
                pathState = 11;
                break;
            case 12:
                follower.follow(park());
                pathState = 12;
                break;*/
            default:
                break;
        }
    }

    @Override
    public void init(){
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
    }

    @Override
    public void start(){
        pathState = 1;
    }

    @Override
    public void loop(){
        follower.update();
        autoPathUpdate();
    }

}



