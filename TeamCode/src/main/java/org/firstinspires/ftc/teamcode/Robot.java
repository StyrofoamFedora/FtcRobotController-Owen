package org.firstinspires.ftc.teamcode;

import android.util.Log; // Import for logging

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Robot {

    private static final String TAG = "Robot"; // Tag for logging

    public DcMotor leftFrontDrive;
    public DcMotor leftBackDrive;
    public DcMotor rightFrontDrive;
    public DcMotor rightBackDrive;
    public DcMotorEx rightFly;
    //public DcMotorEx leftFly;
    public DcMotor topIntake;
    public DcMotor bottomIntake;

    public Servo ethan;  // Servo for Ethan control

    public Robot() {}

    public void initialize(HardwareMap hardwareMap) {
        try {
            leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
            leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
            rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
            rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
            //leftFly = hardwareMap.get(DcMotorEx.class, "leftFly");
            rightFly = hardwareMap.get(DcMotorEx.class, "rightFly");
            topIntake = hardwareMap.get(DcMotor.class, "topIntake");
            bottomIntake = hardwareMap.get(DcMotor.class, "bottomIntake");

            ethan = hardwareMap.get(Servo.class, "ethan");

        } catch (Exception e) {
            Log.e(TAG, "Error initializing hardware", e); // Replaces e.printStackTrace()
        }
    }

    /**
     * Update the drive motors with axial, lateral, yaw and an additional control (set to 0 by default for now)
     * @param axial forward is positive, backward is negative
     * @param lateral left is positive, right is negative
     * @param yaw clockwise is positive, counter-clockwise is negative
     */
    public void updateDriveMotors(double axial, double lateral, double yaw) {
        double max;

        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        double leftFrontPower = axial - lateral + yaw;
        double rightFrontPower = axial - lateral - yaw;
        double leftBackPower = axial + lateral + yaw;
        double rightBackPower = axial + lateral - yaw;

        // Normalize the values so no wheel power exceeds 100%
        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }

    public void updateFlywheelMotors(double power) {
        //leftFly.setPower(1 * power);
        rightFly.setPower(-1 * power);
    }
    public void updateIntakeMotors(double power) {
        topIntake.setPower(1 * power);
        bottomIntake.setPower(1 * power);
    }

    // Update Ethan servo's position (forward or reverse) with clamped values
    public void updateEthanServo(double position) {
        // Clamp the position between some max and min value to prevent it from going too far
        position = Math.max(0.55, Math.min(position, 0.80));
        ethan.setPosition(position);
    }
}
