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
public class AutoThreeFar extends OpMode{
    private Follower follower;
    private final PoseFactory poseFactory = PoseFactory.degrees();
    public int pathState;

    private final Pose start = poseFactory.of(61, 133, 90);
    private final Pose shoot = poseFactory.of(59, 118, 90);
    private final Pose preFlower = poseFactory.of(46, 118, 90);
    private final Pose flower = poseFactory.of(46, 132, 90);
    private final Pose park = poseFactory.of(12, 120, 90);

    public Path start_shoot(){
        return Paths.line(start, shoot).linear(start, shoot);
    }

    public Path shoot_preFlower(){
        return Paths.line(shoot, preFlower).linear(shoot, preFlower);
    }

    public Path preFlower_flower(){
        return Paths.line(preFlower, flower).linear(preFlower, flower);
    }

    public Path flower_shoot(){
        return Paths.line(flower, shoot).linear(flower, shoot);
    }

    public Path shoot_park(){
        return Paths.line(shoot, park).linear(shoot, park);
    }


    public void autoPathUpdate(){
        switch(pathState){
            case 1:
                follower.follow(start_shoot());
                break;
            case 2:
                // shoot
                break;
            case 3:
                follower.follow(shoot_preFlower());
            case 4:
                // flower intake ready
                break;
            case 5:
                follower.follow(preFlower_flower());
            case 6:
                follower.follow(flower_shoot());
            case 7:
                // shoot
                break;
            case 8:
                follower.follow(shoot_park());
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