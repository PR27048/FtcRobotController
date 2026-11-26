package org.firstinspires.ftc.teamcode.pedro;

import org.firstinspires.ftc.teamcode.Teleops.Utilities;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * Auto-only helpers on top of the Teleop Utilities. Everything in Utilities still works
 * (init, HiveUp, Setintake, SetGate, ...), this just adds the auto throw sequence.
 *
 * Throw: call BeginShot() once, then UpdateShot() every loop until it returns true.
 * Fires one pollen at a time: feed until the shooter speed dips (a pollen left),
 * wait for both motors to get back to speed, then feed the next one.
 */
public class AutoUtilities extends Utilities {

    private static final double SHOT_SPEED_TOLERANCE = 40;   // ticks/s, both motors must be within this
    private static final double SHOT_SETTLE_TIME = 0.25;     // s at speed before the first pollen
    private static final double SHOT_SPINUP_TIMEOUT = 1.5;   // s, feed anyway if never at speed
    private static final double SHOT_DIP = 80;               // ticks/s drop that means a pollen just left
    private static final double SHOT_RECOVER_TIMEOUT = 0.75; // s, feed the next one anyway if slow to recover
    private static final double SHOT_EMPTY_TIME = 1.2;       // s feeding with no pollen leaving = robot is empty
    private static final double SHOT_TIMEOUT = 4.5;          // s max from first feed to done
    private static final double SHOT_SPIN_DOWN = 0.3;        // s to keep spinning after the last pollen
    private static final int POLLEN_PER_SHOT = 4;

    private static final int SHOT_IDLE = 0, SHOT_SPINUP = 1, SHOT_FEED = 2, SHOT_RECOVER = 3, SHOT_FINISH = 4;

    // Same motor objects Utilities uses, Utilities keeps its own private
    private DcMotorEx Shooter1, Shooter2;

    private int shotPhase = SHOT_IDLE;
    private int shotCount = 0;
    private double shotVelocity = 0;
    private double feedStartVelocity = 0;
    private final ElapsedTime phaseTimer = new ElapsedTime();
    private final ElapsedTime settleTimer = new ElapsedTime();
    private final ElapsedTime shotTotalTimer = new ElapsedTime();
    private final ElapsedTime lastShotTimer = new ElapsedTime();
    private boolean settling = false;

    @Override
    public void init(HardwareMap hwMap, String autoState) {
        super.init(hwMap, autoState);
        Shooter1 = hwMap.get(DcMotorEx.class, "Shooter1");
        Shooter2 = hwMap.get(DcMotorEx.class, "Shooter2");
    }

    public double getShooter2Velocity() {
        return Shooter2.getVelocity();
    }

    // Both shooter motors within tolerance of the target, not just Shooter1
    public boolean ShootersAtSpeed(double velocity) {
        return Math.abs(Shooter1.getVelocity() - velocity) < SHOT_SPEED_TOLERANCE
                && Math.abs(Shooter2.getVelocity() - velocity) < SHOT_SPEED_TOLERANCE;
    }

    public void BeginShot(double fallbackVelocity) {
        shotVelocity = fallbackVelocity;
        shotCount = 0;
        settling = false;
        shotPhase = SHOT_SPINUP;
        phaseTimer.reset();
    }

    // Returns true once the throw is finished and the shooter is stopped
    public boolean UpdateShot() {
        if (shotPhase == SHOT_IDLE) {
            return true;
        }

        // Aim speed from the Limelight when it sees the hive, else keep the last/fallback speed.
        // HiveUp() null-checks the result, GetDistance() does not.
        if (HiveUp()) {
            double x = GetDistance();
            if (!Double.isNaN(x) && x > 0) {
                shotVelocity = 1450; //VelocityFromDistance(x);
            }
        }
        SetShooterVelocity(shotVelocity);
        SetGate(0.69);

        double slowest = Math.min(Shooter1.getVelocity(), Shooter2.getVelocity());

        switch (shotPhase) {
            case SHOT_SPINUP: // wait until both motors have been at speed for a moment
                if (ShootersAtSpeed(shotVelocity)) {
                    if (!settling) {
                        settling = true;
                        settleTimer.reset();
                    }
                } else {
                    settling = false;
                }
                if ((settling && settleTimer.seconds() >= SHOT_SETTLE_TIME)
                        || phaseTimer.seconds() >= SHOT_SPINUP_TIMEOUT) {
                    shotTotalTimer.reset();
                    StartFeeding(slowest);
                }
                break;

            case SHOT_FEED: // push pollen in until the speed dips
                if (SlingerUp()) {
                    Setintake(0.7, 0.4); // slower transfer spaces the pollen out
                    SetFeeder(1.0);
                } else {
                    Setintake(0.0, 0.0);
                    SetFeeder(0.0);
                }

                if (feedStartVelocity - slowest > SHOT_DIP) {
                    shotCount++;
                    Setintake(0.0, 0.0);
                    SetFeeder(0.0);
                    shotPhase = SHOT_RECOVER;
                    phaseTimer.reset();
                } else if (lastShotTimer.seconds() >= SHOT_EMPTY_TIME
                        || shotTotalTimer.seconds() >= SHOT_TIMEOUT) {
                    FinishShot();
                }
                break;

            case SHOT_RECOVER: // nothing feeds until both motors are back to speed
                Setintake(0.0, 0.0);
                SetFeeder(0.0);
                if (shotCount >= POLLEN_PER_SHOT || shotTotalTimer.seconds() >= SHOT_TIMEOUT) {
                    FinishShot();
                } else if (ShootersAtSpeed(shotVelocity) || phaseTimer.seconds() >= SHOT_RECOVER_TIMEOUT) {
                    StartFeeding(slowest);
                }
                break;

            case SHOT_FINISH: // let the last pollen clear before stopping the wheels
                Setintake(0.0, 0.0);
                SetFeeder(0.0);
                if (phaseTimer.seconds() >= SHOT_SPIN_DOWN) {
                    StopShooting();
                    return true;
                }
                break;
        }
        return false;
    }

    private void StartFeeding(double currentVelocity) {
        feedStartVelocity = currentVelocity; // dips are measured from here, not from the target
        lastShotTimer.reset();
        shotPhase = SHOT_FEED;
        phaseTimer.reset();
    }

    private void FinishShot() {
        shotPhase = SHOT_FINISH;
        phaseTimer.reset();
    }

    public void StopShooting() {
        shotPhase = SHOT_IDLE;
        Setintake(0.0, 0.0);
        SetFeeder(0.0);
        SetGate(0.07);
        SetShooterVelocity(0);
    }

    public double GetShotVelocity() {
        return shotVelocity;
    }

    public int GetShotCount() {
        return shotCount;
    }
}
