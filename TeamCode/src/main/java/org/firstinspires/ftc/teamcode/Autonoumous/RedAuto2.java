package org.firstinspires.ftc.teamcode.Autonoumous;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.Teleops.Utilities;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import static com.pedropathing.api.Paths.*;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static com.pedropathing.ivy.Scheduler.schedule;
//import java.nio.file.Path;


@Autonomous
public class RedAuto2 extends OpMode {
    Utilities utils = new Utilities();
    private Follower follower;
    private final PoseFactory p = PoseFactory.degrees();

    // all poses

    private final Pose startPos = p.of(56, 8, 90);
    private final Pose closeShootPos = p.of(56, 24, 90);
    private final Pose prePickUpBalls = p.of(27.8, 8.4, 0);
    private final Pose pickUpBalls = p.of(8.6, 8.4, 0);
    private final Pose farShootPos = p.of(58, 120, 270);

    // all paths

    private Path GoShootOne() {
        return line(startPos, closeShootPos).linear(startPos, closeShootPos);
    }

    private Path PrePickUpPollen() {
        return line(closeShootPos, prePickUpBalls).linear(closeShootPos, prePickUpBalls);
    }

    private Path PickUpPollen() {
        return line(prePickUpBalls, pickUpBalls).linear(prePickUpBalls, pickUpBalls);
    }

    private Path GoShootAgain() {
        return line(pickUpBalls, closeShootPos).linear(pickUpBalls, closeShootPos);
    }

    private Path CloseToFar() {
        return line(closeShootPos, farShootPos).linear(closeShootPos, farShootPos);
    }

    private Path FarToClose() {
        return line(farShootPos, closeShootPos).linear(farShootPos, closeShootPos);
    }

//    private Command IntakePollen(){
//        utils.Setintake(0.7);
//        return null;
//    }
//
//    private Command StopIntakePollen(){
//        utils.Setintake(0);
//        return null;
//    }

//    private boolean checkShoot1(){
//        if(utils.IsAligned()) {
//            utils.SetGate(0.7);
//        }
//        else{
//            follow(follower, CloseToFar());
//        }
//    }

    private Command autoRoutine() {
        return sequential(
                follow(follower, GoShootOne()),
                follow(follower, PrePickUpPollen()),
//                IntakePollen(),
                follow(follower, PickUpPollen()),
                follow(follower, GoShootAgain())
//                StopIntakePollen()
        );
    }

    @Override
    public void init() {
        Scheduler.reset();

        follower = Constants.create(hardwareMap);
        follower.setPose(startPos); // the starting position
        follower.update();
    }
    @Override
    public void start() {
        schedule(autoRoutine());
    }
    @Override
    public void loop() {
        follower.update();
        Scheduler.execute();
    }
}
