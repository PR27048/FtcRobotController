package org.firstinspires.ftc.teamcode.pedro;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


import com.pedropathing.follower.Follower;
import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;


@Autonomous(name = "AutoTipPlus4Pollen")
public class AutoTipPlus4Pollen extends OpMode {


    private final PoseFactory poseFactory = PoseFactory.degrees();


    private final Pose start = poseFactory.of(55.8026, 9.1841, 180);
    private final Pose path1End = poseFactory.of(7.8466, 8.963, 180);
    private final Pose point2Start = poseFactory.of(7.8466, 8.963, 180);
    private final Pose point2 = poseFactory.of(36.3617, 121.5677, 180);
    private final Pose point3Start = poseFactory.of(36.3617, 121.5677, 180);
    private final Pose point3 = poseFactory.of(56.5244, 121.181, 270);
    private final Pose point4Start = poseFactory.of(56.5244, 121.181, 0);
    private final Pose point4 = poseFactory.of(7.9523, 121.6903, 0);


    private Follower follower;
    private final ElapsedTime timer = new ElapsedTime();


    private Path firstPath;
    private Path secondPath;
    private Path thirdPath;
    private Path fourthPath;


    private int pathState = 0;


    public Path path1() {
        return Paths.line(start, path1End).linear(start, path1End);
    }


    public Path path2() {
        return Paths.line(point2Start, point2).linear(point2Start, point2);
    }


    public Path path3() {
        return Paths.line(point3Start, point3).linear(point3Start, point3);
    }


    public Path path4() {
        return Paths.line(point4Start, point4).linear(point4Start, point4);
    }


    @Override
    public void init() {
        follower = Constants.create(hardwareMap);


        firstPath = path1();
        secondPath = path2();
        thirdPath = path3();
        fourthPath = path4();
    }


    @Override
    public void start() {
        timer.reset();
        pathState = 1;
    }


    @Override
    public void loop() {
        follower.update();


        switch (pathState) {
            case 1:
                if (timer.seconds() >= 2.0) {
                    follower.follow(firstPath);
                    pathState = 2;
                }
                break;


            case 2:
                if (!follower.isBusy()) {
                    timer.reset();
                    pathState = 3;
                }
                break;


            case 3:
                if (timer.seconds() >= 2.0) {
                    follower.follow(secondPath);
                    pathState = 4;
                }
                break;


            case 4:
                if (!follower.isBusy()) {
                    timer.reset();
                    pathState = 5;
                }
                break;


            case 5:
                if (timer.seconds() >= 2.0) {
                    follower.follow(thirdPath);
                    pathState = 6;
                }
                break;


            case 6:
                if (!follower.isBusy()) {
                    timer.reset();
                    pathState = 7;
                }
                break;


            case 7:
                if (timer.seconds() >= 2.0) {
                    pathState = 8;
                }
                break;


            case 8:
                if (timer.seconds() >= 2.0) {
                    follower.follow(fourthPath);
                    pathState = 9;
                }
                break;


            case 9:
                if (!follower.isBusy()) {
                    timer.reset();
                    pathState = 10;
                }
                break;


            case 10:
                break;
        }
    }
}
