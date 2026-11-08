package org.firstinspires.ftc.teamcode.TuningPrograms;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Servo Positional Tester", group = "Tuning")
public class ServoTester extends OpMode {

    public Servo FrontalSweep;
    public double targetPos = 0.0;

    // Debounce state tracking
    private boolean lastDpadUp = false;
    private boolean lastDpadRight = false;
    private boolean lastDpadDown = false;
    private boolean lastDpadLeft = false;
    private boolean lastY = false;

    @Override
    public void init() {
        FrontalSweep = hardwareMap.get(Servo.class, "FrontalSweep");
        targetPos = 0.0;
        FrontalSweep.setPosition(targetPos);
    }

    @Override
    public void loop() {
        boolean currUp = gamepad1.dpad_up;
        boolean currRight = gamepad1.dpad_right;
        boolean currDown = gamepad1.dpad_down;
        boolean currLeft = gamepad1.dpad_left;
        boolean currY = gamepad1.y;

        // --- POSITIONAL PRESETS (Jump directly to a specific absolute angle) ---

        if (currUp && !lastDpadUp) {
            targetPos = 0.0;  // 0 degrees (0 turns)
        }
        if (currRight && !lastDpadRight) {
            targetPos = 0.2;  // 72 degrees (1 turn)
        }
        if (currDown && !lastDpadDown) {
            targetPos = 0.5;  // 180 degrees (2.5 turns)
        }
        if (currLeft && !lastDpadLeft) {
            targetPos = 0.8;  // 288 degrees (4 turns)
        }
        if (currY && !lastY) {
            targetPos = 1.0;  // 360 degrees (5 turns - Full Circle)
        }

        // Save states for next loop
        lastDpadUp = currUp;
        lastDpadRight = currRight;
        lastDpadDown = currDown;
        lastDpadLeft = currLeft;
        lastY = currY;

        // Command the servo to lock into that absolute position
        FrontalSweep.setPosition(targetPos);

        // Telemetry
        telemetry.addData("Commanded Target Pos", targetPos);
        telemetry.addData("Actual Servo Feedback", FrontalSweep.getPosition());
        telemetry.update();
    }
}