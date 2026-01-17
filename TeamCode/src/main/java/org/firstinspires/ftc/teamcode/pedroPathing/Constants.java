package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
//import org.firstinspires.ftc.teamcode.localization.constants;
import org.firstinspires.ftc.teamcode.drivetrains.*;
import org.firstinspires.ftc.teamcode.localization.constants.DriveEncoderConstants;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(5.1) //this has to be in killogram
            .forwardZeroPowerAcceleration(-26.999) //get this by running pedropath test (i have picked a number for now)
            .lateralZeroPowerAcceleration(-92.2222)
            //.translationalPIDFCoefficiants(new PIDFCoefficients(0.06,0,0.0001,0.025)) //set using testing
            //.headingPIDFCoefficiants(new PIDFCoefficients(0.71,0,0.0002,0.025)) // set by testing
           // .centripetalScaling(0.0005) //set by testing
            // .drivePIDFCoefficiants(new FilteredPIDFCoefficients(0.06,0,0.0001,0.025)) //set by testing

            ;
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("front_right")
            .rightRearMotorName("back_right")
            .leftRearMotorName("back_left")
            .leftFrontMotorName("front_left")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(67.001) //get this by running pedropath test (i have picked a number for now)
            .yVelocity(24.55);

    //goto https://pedropathing.com/docs/pathing/tuning/localization/drive-encoder to see details
    //In the tuning OpMode, under localization, select and start the forward tuner. Then, push the robot forward 48 inches (exactly 2 field tiles). This distance is configurable if needed. Once you push the robot forward, two numbers will be displayed on telemetry:
    //The distance the robot thinks it has traveled
    //The multiplier; this is the number you want.
    public static double fwdMultiplier = 1; //defaulting to 1

    //The lateral tuner is very similar to the forward tuner, except it is sideways. In the tuning OpMode, under localization, select and start the lateral tuner. Push the robot left 48 inches (exactly 2 field tiles). As with the forward tuner, this distance is configurable.
    //Lastly, add the multiplier to DriveEncoderConstants by adding the following line.
    public static double latMultiplier = 1; //defaulting to 1

    //The turn tuner is again, similar to both the forward tuner and lateral tuner, except it is rotational. Place the robot so it aligns to a fixed reference point (eg. edge of a field tile). In the tuning OpMode, under localization, select and start the turn tuner. Rotate the robot counterclockwise one full rotation. As with the previous tuners, this amount is configurable.
    //Lastly, add the multiplier to DriveEncoderConstants by adding the following line.
    public static double turnMultiplier = 1; //defaulting to 1

    public static DriveEncoderConstants localizerConstants = new DriveEncoderConstants()
            .robotWidth(18) //Width: the distance between the left and right wheels
            .robotLength(18) //Length: the distance between the front and back wheels
            .forwardTicksToInches(fwdMultiplier)
            .strafeTicksToInches(latMultiplier)
            .turnTicksToInches(turnMultiplier)
            .rightFrontMotorName("front_right")
            .rightRearMotorName("back_right")
            .leftRearMotorName("back_left")
            .leftFrontMotorName("front_left")
            .leftFrontEncoderDirection(Encoder.REVERSE)
            .leftRearEncoderDirection(Encoder.REVERSE)
            .rightFrontEncoderDirection(Encoder.FORWARD)
            .rightRearEncoderDirection(Encoder.FORWARD);
    public static PathConstraints pathConstraints = new PathConstraints(0.99,
            100,
            1,
            1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
               // .driveEncoderLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
             //   .mechanumDrivetrain(driveConstants)

                .build();
    }
}
