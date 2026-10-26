package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import static com.pedropathing.api.Paths.*;

import com.pedropathing.ivy.Scheduler;
import com.pedropathing.ivy.Command;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.commands.Commands.*;


import com.pedropathing.paths.Path;

@Autonomous(name="JyothiAutoTest")
public class JyothiAutoTest extends OpMode {

    private Follower follower;
    private final PoseFactory p = PoseFactory.degrees();

    //Poses
    private final Pose start = p.of(36.8894, 12.0661, 0);
    private final Pose path1 = p.of(45.7586, 12.0661, 0);
    private final Pose path2 = p.of(55.0000, 22.0000, 0);


    private Path startTopath1() {
        return line(start, path1).linear(start, path1);
    }

    private Path path1ToPath2() {
        return line(path1, path2).linear(path1, path2);
    }

    private Command autoRoutine() {
        return sequential(
                follow(follower, startTopath1()),
                waitMs(1000),
                follow(follower,path1ToPath2())


        );
    }


    @Override
    public void init() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
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

        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData("Follower Mode", follower.mode());
        telemetry.update();

    }
}
