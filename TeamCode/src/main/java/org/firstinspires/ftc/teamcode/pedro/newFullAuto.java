package org.firstinspires.ftc.teamcode.pedro;

import static com.pedropathing.api.Paths.*;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import org.firstinspires.ftc.teamcode.Teleops.Utilities;

import com.pedropathing.api.Paths;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class newFullAuto extends OpMode {

    private Utilities utilities = new Utilities();


    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(56, 8, 270);
    private final Pose path1 = poseFactory.of(56, 20, 270);
    private final Pose point2Start = poseFactory.of(56, 20, 270);
    private final Pose point2 = poseFactory.of(22, 46.6733, 180);
    private final Pose point3Start = poseFactory.of(22, 46.6733, 180);
    private final Pose point3 = poseFactory.of(15, 46.67, 180);
    private final Pose point4Start = poseFactory.of(15, 46.67, 180);
    private final Pose point4 = poseFactory.of(22, 46.67, 180);
    private final Pose point5Start = poseFactory.of(22, 46.67, 180);
    private final Pose point5 = poseFactory.of(56, 120, 90);
    private final Pose point5Control1 = poseFactory.of(26.7903, 114.5617, 0);
    private final Pose point6Start = poseFactory.of(56, 120, 90);
    private final Pose point6 = poseFactory.of(46.5, 120, 90);
    private final Pose point7Start = poseFactory.of(46.5, 120, 90);
    private final Pose point7 = poseFactory.of(46.624, 127, 90);
    private final Pose point8Start = poseFactory.of(46.624, 127, 90);
    private final Pose point8 = poseFactory.of(56, 120, 90);
    private final Pose point9Start = poseFactory.of(56, 120, 90);
    private final Pose point9 = poseFactory.of(10, 23.2873, 270);
    private final Pose point9Control1 = poseFactory.of(1.83, 100, 0);
    private final Pose point9Control2 = poseFactory.of(41.5915, 34.8323, 0);
    private final Pose point10Start = poseFactory.of(10, 23.2873, 270);
    private final Pose point10 = poseFactory.of(10, 9.5, 270);
    private final Pose point11Start = poseFactory.of(10, 9.5, 270);
    private final Pose point11 = poseFactory.of(56, 20, 270);
    private final Pose point12Start = poseFactory.of(56, 20, 270);
    private final Pose point12 = poseFactory.of(9.9168, 106.4704, 270);
    private final Pose point12Control1 = poseFactory.of(17.6135, 33.0561, 0);

    private Follower follower;
    private final ElapsedTime timer = new ElapsedTime();

    private Path firstPath;
    private Path secondPath;
    private Path thirdPath;
    private Path fourthPath;
    private Path fifthPath;
    private Path sixthPath;
    private Path seventhPath;
    private Path eighthPath;
    private Path ninthPath;
    private Path tenthPath;
    private Path eleventhPath;
    private Path twelfthPath;

    private int pathState = 0;

    public Path path1() {
        return Paths.line(start, path1).linear(start, path1);
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

    public Path path5() {
        return Paths.curve(point5Start, point5Control1, point5).linear(point5Start, point5);
    }

    public Path path6() {
        return Paths.line(point6Start, point6).linear(point6Start, point6);
    }

    public Path path7() {
        return Paths.line(point7Start, point7).linear(point7Start, point7);
    }

    public Path path8() {
        return Paths.line(point8Start, point8).linear(point8Start, point8);
    }

    public Path path9() {
        return Paths.curve(point9Start, point9Control1, point9Control2, point9).linear(point9Start, point9);
    }

    public Path path10() {
        return Paths.line(point10Start, point10).linear(point10Start, point10);
    }

    public Path path11() {
        return Paths.line(point11Start, point11).linear(point11Start, point11);
    }

    public Path path12() {
        return Paths.curve(point12Start, point12Control1, point12).linear(point12Start, point12);
    }

    @Override
    public void init() {

        utilities.init(hardwareMap, "auto");

        follower = Constants.create(hardwareMap);
        follower.setPose(start);

        firstPath = path1();
        secondPath = path2();
        thirdPath = path3();
        fourthPath = path4();
        fifthPath = path5();
        sixthPath = path6();
        seventhPath = path7();
        eighthPath = path8();
        ninthPath = path9();
        tenthPath = path10();
        eleventhPath = path11();
        twelfthPath = path12();
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
                if (timer.seconds() >= 0.75) {
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
                if (timer.seconds() >= 1) {
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
                if (timer.seconds() >= 0.5) {
                    follower.follow(thirdPath);
                    pathState = 6;
                }break;

            case 6:
                if (!follower.isBusy()) {
                    follower.follow(fourthPath);
                    pathState = 7;
                }
                break;

            case 7:
                if (!follower.isBusy()) {
                    follower.follow(fifthPath);
                    pathState = 8;
                }
                break;

            case 8:
                if (!follower.isBusy()) {
                    timer.reset();
                    pathState = 9;
                }
                break;

            case 9:
                if (timer.seconds() >= 1) {
                    follower.follow(sixthPath);
                    pathState = 10;
                }
                break;

            case 10:
                if (!follower.isBusy()) {
                    timer.reset();
                    pathState = 11;
                }
                break;

            case 11:
                if (timer.seconds() >= 1) {
                    follower.follow(seventhPath);
                    pathState = 12;
                }
                break;

            case 12:
                if (!follower.isBusy()) {
                    follower.follow(eighthPath);
                    pathState = 13;
                }
                break;

            case 13:
                if (!follower.isBusy()) {
                    timer.reset();
                    pathState = 14;
                }
                break;

            case 14:
                if (timer.seconds() >= 1) {
                    follower.follow(ninthPath);
                    pathState = 15;
                }
                break;

            case 15:
                if (!follower.isBusy() ) {
                    follower.follow(tenthPath);
                    pathState = 16;
                }
                break;

            case 16:
                if (!follower.isBusy()) {
                    follower.follow(eleventhPath);
                    pathState = 17;
                }
                break;

            case 17:
                if (!follower.isBusy()) {
                    timer.reset();
                    pathState = 18;
                }
                break;

            case 18:
                if (timer.seconds() >= 1) {
                    follower.follow(twelfthPath);
                }
                break;

        }

    }
}
