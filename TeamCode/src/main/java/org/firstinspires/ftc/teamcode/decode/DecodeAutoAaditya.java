package org.firstinspires.ftc.teamcode.decode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous
public class DecodeAutoAaditya extends OpMode {

    /// ///////////////////////
    final double FEED_TIME = 0.20;
   // final double LAUNCHER_TARGET_VELOCITY = 1125;
    //final double LAUNCHER_MIN_VELOCITY = 1075;
    final double TIME_BETWEEN_SHOTS = 2;
    final double DRIVE_SPEED = 0.5;
    final double ROTATE_SPEED = 0.2;

    double robotRotationAngle = 45;
    //////////////////////////////
    private enum LaunchState {
        IDLE,
        PREPARE,
        LAUNCH,
    }

    private LaunchState launchState;
    int shotsToFire = 3; //The number of shots to fire in this auto.

    // Here is our auto state machine enum. This captures each action we'd like to do in auto.
    private enum AutonomousState {
        LAUNCH,
        WAIT_FOR_LAUNCH,
        DRIVING_AWAY_FROM_GOAL,
        ROTATING,
        DRIVING_OFF_LINE,
        COMPLETE
    }

    private AutonomousState autonomousState;

    //Here we create an enum not to create a state machine, but to capture which alliance we are on.
    private enum Alliance {
        RED,
        BLUE
    }

    private ElapsedTime shotTimer = new ElapsedTime();
    private ElapsedTime feederTimer = new ElapsedTime();
  //  private ElapsedTime driveTimer = new ElapsedTime();

    /*
     * When we create the instance of our enum we can also assign a default state.
     */
    private Alliance alliance = Alliance.RED;

    ServiceHelperAaditya serviceHelper = new ServiceHelperAaditya();

    // This code runs ONCE when the driver hits INIT.
    @Override
    public void init(){

        //Here we set the first step of our autonomous state machine by setting autoStep = AutoStep.LAUNCH.
        autonomousState = AutonomousState.LAUNCH;
        launchState = LaunchState.IDLE;

        //initialize hardware (drivetrain
        serviceHelper.init(hardwareMap, "TRUE");

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");

    }

    //This code runs REPEATEDLY after the driver hits INIT, but before they hit START.
    @Override
    public void init_loop() {

        /*
         * Here we allow the driver to select which alliance we are on using the gamepad.
         */
        if (gamepad1.b) {
            alliance = Alliance.BLUE;
        } else if (gamepad1.x) {
            alliance = Alliance.RED;
        }

        telemetry.addData("Press X", "for RED");
        telemetry.addData("Press B", "for BLUE");
        telemetry.addData("Selected Alliance", alliance);
    }

    @Override
    public void loop(){
        switch (autonomousState) {
            case LAUNCH:
                launch(true);
                autonomousState = AutonomousState.WAIT_FOR_LAUNCH;
                break;

            case WAIT_FOR_LAUNCH:

                if(launch(false)) {
                    shotsToFire -= 1;
                    if(shotsToFire > 0) {
                        autonomousState = AutonomousState.LAUNCH;
                    } else {
                        autonomousState = AutonomousState.DRIVING_AWAY_FROM_GOAL;
                    }
                }
                break;
            case DRIVING_AWAY_FROM_GOAL:
                /*
                 * This is another function that returns a boolean. This time we return "true" if
                 * the robot has been within a tolerance of the target position for "holdSeconds."
                 * Once the function returns "true" we reset the encoders again and move on.
                 */
                 if(serviceHelper.driveToPosition(DRIVE_SPEED, -4, DistanceUnit.INCH, 1)){

                    autonomousState = AutonomousState.ROTATING;
                }
                break;
            case ROTATING:
                if(alliance == Alliance.RED){
                    robotRotationAngle = 45;
                } else if (alliance == Alliance.BLUE){
                    robotRotationAngle = -45;
                }

              //  if(rotate(ROTATE_SPEED, robotRotationAngle, AngleUnit.DEGREES,1)){}

                    autonomousState = AutonomousState.DRIVING_OFF_LINE;

                break;
            case DRIVING_OFF_LINE:
                if(serviceHelper.driveToPosition(DRIVE_SPEED, -26, DistanceUnit.INCH, 1)){
                    autonomousState = AutonomousState.COMPLETE;
                }
                break;

        }
        telemetry.addData("AutoState", autonomousState);
        telemetry.addData("LauncherState", launchState);
       /* telemetry.addData("Motor Current Positions", "left (%d), right (%d)",
                leftDrive.getCurrentPosition(), rightDrive.getCurrentPosition());
        telemetry.addData("Motor Target Positions", "left (%d), right (%d)",
                leftDrive.getTargetPosition(), rightDrive.getTargetPosition());*/
        telemetry.update();

    }

    /**
     * Launches one ball, when a shot is requested spins up the motor and once it is above a minimum
     * velocity, runs the feeder servos for the right amount of time to feed the next ball.
     * @param shotRequested "true" if the user would like to fire a new shot, and "false" if a shot
     *                      has already been requested and we need to continue to move through the
     *                      state machine and launch the ball.
     * @return "true" for one cycle after a ball has been successfully launched, "false" otherwise.
     */
    boolean launch(boolean shotRequested){
        switch (launchState) {
            case IDLE:
                if (shotRequested) {
                    launchState = LaunchState.PREPARE;
                    shotTimer.reset();
                }
                break;
            case PREPARE:
                serviceHelper.SetTurretPower();
                serviceHelper.SetIntakePower(0);
                serviceHelper.SetServoConFrontPower(-1.0);
                serviceHelper.setFeederPower(0.7);
                serviceHelper.SetServoConFrontPower(-1.0);
                serviceHelper.setIntakeServoPower(-1.0);
                serviceHelper.SetTurretPower();
                launchState = LaunchState.LAUNCH;
                feederTimer.reset();

                break;
            case LAUNCH:
                if (feederTimer.seconds() > FEED_TIME) {
                    serviceHelper.SetIntakePower(0.0);
                    //drive.SetTurretPower(0.0);
                    serviceHelper.SetServoConFrontPower(0.0);
                    serviceHelper.setServoConPower(0.0);
                    serviceHelper.setFeederPower(0.0);
                    serviceHelper.SetServoConFrontPower(0.0);
                    serviceHelper.setIntakeServoPower(0.0);

                    if(shotTimer.seconds() > TIME_BETWEEN_SHOTS){
                        launchState = LaunchState.IDLE;
                        return true;
                    }
                }
        }
        return false;
    }

}
