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
public class FullAuto extends OpMode {

    private Utilities utilities = new Utilities();

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(56, 9.5, 90);
    private final Pose point1 = poseFactory.of(56, 15, 90);
    private final Pose point2Start = poseFactory.of(56, 15, 90);
    private final Pose point2 = poseFactory.of(9.5, 15, 270);
    private final Pose point3Start = poseFactory.of(9.5, 15, 270);
    private final Pose point3 = poseFactory.of(9, 9, 270);
    private final Pose point4Start = poseFactory.of(9, 9, 270);
    private final Pose point4 = poseFactory.of(58, 26.5436, 90);
    private final Pose point5Start = poseFactory.of(58, 26.5436, 90);
    private final Pose point5 = poseFactory.of(58, 119.1994, 90);
    private final Pose point6Start = poseFactory.of(58, 119.1994, 90);
    private final Pose point6 = poseFactory.of(51.0928, 119.0427, 180);
    private final Pose point7Start = poseFactory.of(51.0928, 119.0427, 180);
    private final Pose point7 = poseFactory.of(58, 119.2, 270);
    private final Pose point8Start = poseFactory.of(58, 119.2, 270);
    private final Pose point8 = poseFactory.of(45.4399, 119.2, 90);
    private final Pose point9Start = poseFactory.of(45.4399, 119.2, 90);
    private final Pose point9 = poseFactory.of(45.44, 127, 90);
    private final Pose point10Start = poseFactory.of(45.44, 127, 90);
    private final Pose point10 = poseFactory.of(58, 119.2, 270);
    private final Pose point11Start = poseFactory.of(58, 119.2, 270);
    private final Pose point11 = poseFactory.of(22.3499, 119.2, 270);
    private final Pose point12Start = poseFactory.of(22.3499, 119.2, 270);
    private final Pose point12 = poseFactory.of(22.35, 46.6733, 180);
    private final Pose point13Start = poseFactory.of(22.35, 46.6733, 180);
    private final Pose point13 = poseFactory.of(14.5, 46.9693, 180);
    private final Pose point14Start = poseFactory.of(14.5, 46.9693, 180);
    private final Pose point14 = poseFactory.of(45.44, 119.2, 270);
    private final Pose point15Start = poseFactory.of(45.44, 119.2, 270);
    private final Pose point15 = poseFactory.of(58, 119.2, 270);
    private final Pose point16Start = poseFactory.of(58, 119.2, 270);
    private final Pose point16 = poseFactory.of(56, 15, 270);
    private final Pose point17Start = poseFactory.of(56, 15, 270);
    private final Pose point17 = poseFactory.of(50, 15, 180);
    private final Pose point18Start = poseFactory.of(50, 15, 180);
    private final Pose point18 = poseFactory.of(56, 15, 90);
    private final Pose point19Start = poseFactory.of(56, 15, 90);
    private final Pose point19 = poseFactory.of(10, 70, 90);
    private final Pose point20Start = poseFactory.of(10, 70, 90);
    private final Pose point20 = poseFactory.of(10, 110, 90);
    private final Pose point21Start = poseFactory.of(58, 119.29, 270);
    private final Pose point21 = poseFactory.of(12, 119.29, 270);
    private final Pose point22Start = poseFactory.of(58.465, 26.5436, 90);
    private final Pose point22 = poseFactory.of(21.7578, 46.6733, 180);
    private final Pose point23Start = poseFactory.of(21.7578, 46.6733, 0);
    private final Pose point23 = poseFactory.of(14.5, 46.67, 0);
    private final Pose point24Start = poseFactory.of(14.5, 46.67, 180);
    private final Pose point24 = poseFactory.of(56, 15, 90);
    private final Pose point25Start = poseFactory.of(56, 15, 90);
    private final Pose point25 = poseFactory.of(59, 15, 90);
    private final Pose point26Start = poseFactory.of(59, 15, 90);
    private final Pose point26 = poseFactory.of(59, 119.2, 90);
    private final Pose point27Start = poseFactory.of(59, 119.2, 90);
    private final Pose point27 = poseFactory.of(56, 119.2, 270);
    private final Pose point28Start = poseFactory.of(56, 119.2, 270);
    private final Pose point28 = poseFactory.of(47, 119.2, 90);
    private final Pose point29Start = poseFactory.of(47, 119.2, 90);
    private final Pose point29 = poseFactory.of(47.068, 127, 90);
    private final Pose point30Start = poseFactory.of(47.068, 127, 90);
    private final Pose point30 = poseFactory.of(47, 119.2, 90);
    private final Pose point31Start = poseFactory.of(47, 119.2, 90);
    private final Pose point31 = poseFactory.of(56, 119.2, 270);
    private final Pose point32Start = poseFactory.of(56, 119.2, 270);
    private final Pose point32 = poseFactory.of(11.5, 120.0875, 270);

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
    private Path thirteenthPath;
    private Path fourteenthPath;
    private Path fifteenthPath;
    private Path sixteenthPath;
    private Path seventeenthPath;
    private Path eighteenthPath;
    private Path nineteenthPath;
    private Path twentiethPath;
    private Path twentyfirstPath;
    private Path twentysecondPath;
    private Path twentythirdPath;
    private Path twentyfourthPath;
    private Path twentyfifthPath;
    private Path twentysixthPath;
    private Path twentyseventhPath;
    private Path twentyeighthPath;
    private Path twentyninthPath;
    private Path thirtiethPath;
    private Path thirtyfirstPath;
    private Path thirtysecondPath;

    private int pathState = 0;


    public Path path1() {
        return Paths.line(start, point1).linear(start, point1);
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
        return Paths.line(point5Start, point5).linear(point5Start, point5);
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
        return Paths.line(point9Start, point9).linear(point9Start, point9);
    }

    public Path path10() {
        return Paths.line(point10Start, point10).linear(point10Start, point10);
    }

    public Path path11() {
        return Paths.line(point11Start, point11).linear(point11Start, point11);
    }

    public Path path12() {
        return Paths.line(point12Start, point12).linear(point12Start, point12);
    }

    public Path path13() {
        return Paths.line(point13Start, point13).linear(point13Start, point13);
    }

    public Path path14() {
        return Paths.line(point14Start, point14).linear(point14Start, point14);
    }

    public Path path15() {
        return Paths.line(point15Start, point15).linear(point15Start, point15);
    }

    public Path path16() {
        return Paths.line(point16Start, point16).linear(point16Start, point16);
    }

    public Path path17() {
        return Paths.line(point17Start, point17).linear(point17Start, point17);
    }

    public Path path18() {
        return Paths.line(point18Start, point18).linear(point18Start, point18);
    }

    public Path path19() {
        return Paths.line(point19Start, point19).linear(point19Start, point19);
    }

    public Path path20() {
        return Paths.line(point20Start, point20).linear(point20Start, point20);
    }

    public Path path21() {
        return Paths.line(point21Start, point21).linear(point21Start, point21);
    }

    public Path path22() {
        return Paths.line(point22Start, point22).linear(point22Start, point22);
    }

    public Path path23() {
        return Paths.line(point23Start, point23).linear(point23Start, point23);
    }

    public Path path24() {
        return Paths.line(point24Start, point24).linear(point24Start, point24);
    }

    public Path path25() {
        return Paths.line(point25Start, point25).linear(point25Start, point25);
    }

    public Path path26() {
        return Paths.line(point26Start, point26).linear(point26Start, point26);
    }

    public Path path27() {
        return Paths.line(point27Start, point27).linear(point27Start, point27);
    }

    public Path path28() {
        return Paths.line(point28Start, point28).linear(point28Start, point28);
    }

    public Path path29() {
        return Paths.line(point29Start, point29).linear(point29Start, point29);
    }

    public Path path30() {
        return Paths.line(point30Start, point30).linear(point30Start, point30);
    }

    public Path path31() {
        return Paths.line(point31Start, point31).linear(point31Start, point31);
    }

    public Path path32() {
        return Paths.line(point32Start, point32).linear(point32Start, point32);
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
        thirteenthPath = path13();
        fourteenthPath = path14();
        fifteenthPath = path15();
        sixteenthPath = path16();
        seventeenthPath = path17();
        eighteenthPath = path18();
        nineteenthPath = path19();
        twentiethPath = path20();
        twentyfirstPath = path21();
        twentysecondPath = path22();
        twentythirdPath = path23();
        twentyfourthPath = path24();
        twentyfifthPath = path25();
        twentysixthPath = path26();
        twentyseventhPath = path27();
        twentyeighthPath  = path28();
        twentyninthPath = path29();
        thirtiethPath = path30();
        thirtyfirstPath = path31();
        thirtysecondPath = path32();

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
/*
            case 2:
                if (!follower.isBusy()) {
                    timer.reset();
                    pathState = 3;
                }
                break;
*/
            case 2:
                if (!follower.isBusy()) {
                    follower.follow(secondPath);
                    pathState = 3;
                }
                break;
/*
            case 3:
                if (!follower.isBusy()) {
                    timer.reset();
                    pathState = 4;
                }
                break;
*/
            case 3:
                if (!follower.isBusy()) {
                    follower.follow(thirdPath);
                    pathState = 4;
                }
                break;
/*
            case 5:
                if (!follower.isBusy()) {
                    timer.reset();
                    pathState = 6;
                }
                break;

            case 6:
                if (timer.seconds() >= 2.0) {
                    pathState = 7;
                }
                break;
*/
            case 4:
                if (!follower.isBusy()) {
                    follower.follow(fourthPath);
                    pathState = 5;
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    timer.reset();
                    pathState = 6;
                }

            case 6:
                if (!follower.isBusy()) {
                    pathState = 7;
                }
                break;

            case 7:
                if (utilities.IsAligned()) {
                    pathState = 35;
                } else {
                    pathState = 8;
                }
                break;

            case 8:
                if (!follower.isBusy()) {
                    timer.reset();
                    pathState = 9;
                }

            case 9:
                if (!follower.isBusy()) {
                    follower.follow(fifthPath);
                    pathState = 10;
                }
                break;

            case 10:
                if (!follower.isBusy()) {
                    follower.follow(sixthPath);
                    pathState = 11;
                }
                break;

            case 11:
                if (!follower.isBusy()) {
                    follower.follow(seventhPath);
                    pathState = 12;
                }
                break;

            case 12:
                if (!follower.isBusy()) {
                    timer.reset();
                    pathState = 13;

                }


            case 13:
                if (timer.seconds() >= 2) {
                    follower.follow(eighthPath);
                    pathState = 14;
                }
                break;

            case 14:
                if (!follower.isBusy()) {
                    timer.reset();
                    pathState = 15;
                }
            case 15:
                if (timer.seconds() >= 1) {
                    follower.follow(ninthPath);
                    pathState = 16;
                }
                break;

            case 16:
                if (!follower.isBusy()) {
                    follower.follow(tenthPath);
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
                if (timer.seconds() >= 2) {
                    follower.follow(eleventhPath);
                    pathState = 19;
                }
                break;

            case 19:
                if (!follower.isBusy()) {
                    follower.follow(twelfthPath);
                    pathState = 20;
                }
                break;

            case 20:
                if (!follower.isBusy()) {
                    timer.reset();
                    pathState = 21;
                }
                break;

            case 21:
                if (timer.seconds() >= 1) {
                    follower.follow(thirteenthPath);
                    pathState = 22;
                }
                break;

            case 22:
                if (!follower.isBusy()) {
                    follower.follow(fourteenthPath);
                    pathState = 23;
                }
                break;

            case 23:
                if (!follower.isBusy()) {
                    follower.follow(fifteenthPath);
                    pathState = 24;
                }
                break;

            case 24:
                if (!follower.isBusy()) {
                    timer.reset();
                    pathState = 25;
                }
                break;

            case 25:
                if (timer.seconds() >= 1) {
                    pathState = 26;
                }
                break;

            case 26:
                if (utilities.IsAligned()) {
                    pathState = 27;
                } else {
                    pathState = 33;
                }
                break;

            case 27:
                if (!follower.isBusy()) {
                    follower.follow(sixteenthPath);
                    pathState = 28;
                }
                break;

            case 28:
                if (!follower.isBusy()) {
                    follower.follow(seventeenthPath);
                    pathState = 29;
                }
                break;

            case 29:
                if (!follower.isBusy()) {
                    follower.follow(eighteenthPath);
                    pathState = 30;
                }
                break;

            case 30:
                if (!follower.isBusy()) {
                    timer.reset();
                    pathState = 31;
                }
                break;

            case 31:
                if (timer.seconds() >= 2) {
                    follower.follow(nineteenthPath);
                    pathState = 32;
                }
                break;

            case 32:
                if (!follower.isBusy()) {
                    follower.follow(twentiethPath);
                }
                break;

            case 33:
                if (!follower.isBusy()) {
                    timer.reset();
                    pathState = 34;
                }
                break;

            case 34:
                if (timer.seconds() >= 2) {
                    follower.follow(twentyfirstPath);
                }
                break;

            case 35:
                if (!follower.isBusy()) {
                    timer.reset();
                    pathState = 36;
                }
                break;

            case 36:
                if (timer.seconds() >= 2) {
                    follower.follow(twentysecondPath);
                    pathState = 37;
                }
                break;

            case 37:
                if (!follower.isBusy()) {
                    timer.reset();
                    pathState = 38;
                }
                break;

            case 38:
                if (timer.seconds() >= 1) {
                    follower.follow(twentythirdPath);
                    pathState = 39;
                }
                break;

            case 39:
                if (!follower.isBusy()) {
                    follower.follow(twentyfourthPath);
                    pathState = 40;
                }
                break;

            case 40:
                if (!follower.isBusy()) {
                    timer.reset();
                    pathState = 41;
                }
                break;

            case 41:
                if (timer.seconds() >= 2) {
                    follower.follow(twentyfifthPath);
                    pathState = 42;
                }
                break;

            case 42:
                if (!follower.isBusy()) {
                    follower.follow(twentysixthPath);
                    pathState = 43;
                }
                break;

            case 43:
                if (!follower.isBusy()) {
                    follower.follow(twentyseventhPath);
                    pathState = 44;
                }
                break;

            case 44:
                if (!follower.isBusy()) {
                    timer.reset();
                    pathState = 45;
                }
                break;

            case 45:
                if (timer.seconds() >= 2) {
                    follower.follow(twentyeighthPath);
                    pathState = 46;
                }
                break;

            case 46:
                if (!follower.isBusy()) {
                    timer.reset();
                    pathState = 47;
                }
                break;

            case 47:
                if (timer.seconds() >= 1) {
                    follower.follow(twentyninthPath);
                    pathState = 48;
                }
                break;

            case 48:
                if (!follower.isBusy()) {
                    follower.follow(thirtiethPath);
                    pathState = 49;
                }
                break;

            case 49:
                if (!follower.isBusy()) {
                    follower.follow(thirtyfirstPath);
                    pathState = 50;
                }
                break;

            case 50:
                if (!follower.isBusy()) {
                    timer.reset();
                    pathState = 51;
                }
                break;

            case 51:
                if (timer.seconds() >= 2) {
                    follower.follow(thirtysecondPath);
                }
                break;
        }
    }
}