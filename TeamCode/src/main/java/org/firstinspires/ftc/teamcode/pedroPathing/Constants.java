package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(5.1) //this has to be in killogram
            .forwardZeroPowerAcceleration(-26.999) //get this by running pedropath test (i have picked a number for now)
            .lateralZeroPowerAcceleration(-92.2222)
            //.translationalPIDFCoefficiants(new PIDFCoefficients(0.06,0,0.0001,0.025)) //set using testing
            //.headingPIDFCoefficiants(new PIDFCoefficients(0.71,0,0.0002,0.025)) // set by testing
             .centripetalScaling(0.0005) //set by testing
            // .drivePIDFCoefficiants(new FilteredPIDFCoefficients(0.06,0,0.0001,0.025)) //set by testing
            ;
    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .build();
    }
}
