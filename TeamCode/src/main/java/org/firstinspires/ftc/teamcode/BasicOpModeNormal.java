package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Driver Op Mode (Tyler's Controls)", group = "Driver Op Mode")
public class BasicOpModeNormal extends LinearOpMode {

    /** @noinspection FieldMayBeFinal*/
    private ElapsedTime runtime = new ElapsedTime();
    private Robot robot;

    private Gamepad driverController;
    //private Gamepad armController;

    private double flywheelControl = 0;
    private double intakeControl = 0;
    private boolean isTheButtonPressed = false;

    @Override
    public void runOpMode() {
        driverController = gamepad1;
        //armController = gamepad2;
        robot = new Robot();
        robot.initialize(hardwareMap);

        // Wait for the robot to start (driver presses PLAY).
        telemetry.addData("Status", "Initialized.");
        telemetry.addData("Controls", "Use the following controls:");
        telemetry.addData("Strafe Left", "Left Trigger");
        telemetry.addData("Strafe Right", "Right Trigger");
        telemetry.addData("Forward/Back", "LEFT Stick up/down");
        telemetry.addData("Rotate", "RIGHT Stick left/right");
        telemetry.addData("Flywheel On", "Push X to Switch on");
        telemetry.addData("Flywheel Off", "Push B to Switch off");
        telemetry.addData("Ethan Servo Control", "D-pad Up: Forward, D-pad Down: Reverse");
        telemetry.addData("Good luck!", "DON'T CRASH THE ROBOT PLS :)");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // Run until the end of the match (driver presses STOP or time runs out).
        while (opModeIsActive()) {
            updateDrive();
            updateFlywheel();
            updateIntake();
            telemetry.addData("Flywheel %",flywheelControl*100);
            updateEthanServo();  // Update Ethan servo control based on D-pad input

            telemetry.update();
        }
    }

    // Update driving controls (tank drive or similar).
    private void updateDrive() {
        double axial = (1.0 * driverController.right_stick_x); // Forward Back
        double lateral = (0.6 * (driverController.left_trigger - driverController.right_trigger)); // Strafing
        double yaw = (-0.6 * driverController.left_stick_y); // Rotate

        robot.updateDriveMotors(axial, lateral, yaw);
    }

    // Update flywheel motors based on X and B button presses
    private void updateFlywheel() {
        // variable control based on buttons pushed
        if (driverController.right_bumper) {
            flywheelControl = 0;
        } else if (driverController.x) {
            flywheelControl = 0.55;
        } else if (driverController.b) {
            flywheelControl = 0.40;
        } else if (driverController.start) {
            if (!isTheButtonPressed) {
               flywheelControl += 0.05;
               isTheButtonPressed = true;
            }
        } else if (driverController.back) {
            if (!isTheButtonPressed) {
                flywheelControl -= 0.05;
                isTheButtonPressed = true;
            }
        } else {
            isTheButtonPressed = false;
        }

        if (flywheelControl != 0) {
            robot.updateFlywheelMotors(flywheelControl);  // variable speed on flywheel depending on button
        } else {
            robot.updateFlywheelMotors(0.0);  // Stop flywheel
        }
    }
    private void updateIntake() {
        if (driverController.y) {
            intakeControl = 1;
        } else if (driverController.a) {
            intakeControl = -1;
        } else {
            intakeControl = 0;
        }
        robot.updateIntakeMotors(intakeControl);

    }

    // Update Ethan servo based on D-pad input
    private void updateEthanServo() {
        if (driverController.dpad_up) {
            robot.updateEthanServo(0.0);  // Move Ethan forward (servo position 1.0)
        } else {
            robot.updateEthanServo(1.0);  // Move Ethan reverse (servo position 0.0)
        }
    }
}
// blah blah blah