package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous
public class AutoCodeTest extends OpMode {
    private enum LaunchState {
        IDLE,
        PREPARE,
        LAUNCH,
    }
    private LaunchState launchState;

    /*
     * Here is our auto state machine enum. This captures each action we'd like to do in auto.
     */
    private enum AutonomousState {
        LAUNCH,
        WAIT_FOR_LAUNCH,
        DRIVING_AWAY_FROM_GOAL,
        ROTATING,
        DRIVING_OFF_LINE,
        COMPLETE;
    }

    private AutonomousState autonomousState;

    /*
     * Here we create an enum not to create a state machine, but to capture which alliance we are on.
     */
    private enum Alliance {
        RED,
        BLUE;
    }

    /*
     * When we create the instance of our enum we can also assign a default state.
     */
    private Alliance alliance = Alliance.RED;
    @Override
    public void init() {
        //telemetry.addData("Hello", "test");

        }
        @Override
        public void init_loop(){
        if(alliance !=null) {
            if (gamepad1.b) {
                alliance = Alliance.RED;
            } else if (gamepad1.x) {
                alliance = Alliance.BLUE;
            }

            telemetry.addData("Press X", "for BLUE");
            telemetry.addData("Press B", "for RED");
            telemetry.addData("Selected Alliance", alliance);
        }

        }
    @Override
    public void loop() {
        if(autonomousState != null) {
            switch (autonomousState) {
                case LAUNCH:
                    autonomousState = AutonomousState.WAIT_FOR_LAUNCH;
                    break;
                case WAIT_FOR_LAUNCH:
                    autonomousState = AutonomousState.LAUNCH;
                    break;
                case DRIVING_AWAY_FROM_GOAL:
                    autonomousState = AutonomousState.ROTATING;
                    break;
                case ROTATING:
                    autonomousState = AutonomousState.DRIVING_OFF_LINE;
                    break;
                case DRIVING_OFF_LINE:
                    autonomousState = AutonomousState.COMPLETE;
                    break;

                }

            telemetry.addData("AutoState", autonomousState);
            }
        }


    }
