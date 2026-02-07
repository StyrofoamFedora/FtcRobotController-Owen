package org.firstinspires.ftc.teamcode;

import android.util.Log; // Import for logging
import android.util.Size;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.teamcode.Webcam;

import java.util.ArrayList;
import java.util.List;

public class Robot {

    private static final String TAG = "Robot"; // Tag for logging

    public DcMotor leftFrontDrive;
    public DcMotor leftBackDrive;
    public DcMotor rightFrontDrive;
    public DcMotor rightBackDrive;
    public DcMotorEx rightFly;
    public DcMotorEx spindexer;
    public DcMotor Intake;
    public Servo kick;
    public Servo RGB1;
    public Servo RGB2;
    public ColorRangeSensor shootColor;
    public DistanceSensor frontDistance;
    public IMU imu;
    public WebcamName eyes;
    public List<AprilTagDetection> detectedTags = new ArrayList<>();
    public double currentTick=0;
    public String shootBall;
    public boolean frontSlot;
    public Robot() {}


    public void initialize(HardwareMap hardwareMap) {
        try {
            leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
            leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
            rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
            rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
            //leftFly = hardwareMap.get(DcMotorEx.class, "leftFly");
            rightFly = hardwareMap.get(DcMotorEx.class, "rightFly");
            Intake = hardwareMap.get(DcMotor.class, "topIntake");
            spindexer = hardwareMap.get(DcMotorEx.class, "bottomIntake");
            imu = hardwareMap.get(IMU.class, "imu");
            // Adjust the orientation parameters to match your robot
            IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.UP,
                    RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
            imu.initialize(parameters);
            kick = hardwareMap.servo.get("ethan");
            RGB1 = hardwareMap.servo.get("rgb");
            RGB2 = hardwareMap.servo.get("rgb2");
            shootColor = hardwareMap.get(ColorRangeSensor.class, "color");
            frontDistance = hardwareMap.get(DistanceSensor.class,"frontSlot");
            eyes = hardwareMap.get(WebcamName.class,"eyes");
            rightFly.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            spindexer.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            spindexer.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            spindexer.setTargetPositionTolerance(2);
            spindexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            spindexer.setTargetPosition(0);
            leftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
            leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);
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
        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        // Rotate the movement direction counter to the bot's rotation
        double rotX = axial * Math.cos(-botHeading) - lateral * Math.sin(-botHeading);
        double rotY = axial * Math.sin(-botHeading) + lateral * Math.cos(-botHeading);
        rotX = rotX * 1.1;  // Counteract imperfect strafing
        // Normalize the values so no wheel power exceeds 100%
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(yaw), 1);
        double leftFrontPower = (rotY + rotX + yaw) / denominator;
        double leftBackPower = (rotY - rotX + yaw) / denominator;
        double rightFrontPower = (rotY - rotX - yaw) / denominator;
        double rightBackPower = (rotY + rotX - yaw) / denominator;
        //Drive
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }
    public void updateFlywheelMotors(double Velocity) {
        double F = 0.007*(Velocity+4.8);
        double P = 75;
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P,0,0,F);
        rightFly.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        rightFly.setVelocity(Velocity);
    }
    public void updateIntakeMotors(double power) {
        Intake.setPower(1 * power);
    }

    // Update Ethan servo's position (forward or reverse) with clamped values
    public void updateKickServo(double position) {
        // Clamp the position between some max and min value to prevent it from going too far
        kick.setPosition(position);
    }
    public void updateSpindexSlot(double slots, double power){
        double CPR = 537.7;
        double slotTicks = CPR/3;
        currentTick +=(slotTicks*slots);
        int currentIntTick = (int)Math.round(currentTick);
        spindexer.setTargetPosition(currentIntTick);
        spindexer.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        spindexer.setPower(power);
    }
    public void updateColorDistance(){
        double hue = JavaUtil.colorToHue(shootColor.getNormalizedColors().toColor());
        if(hue < 80){shootBall = "None"; }
        else if (hue < 200) {shootBall = "Green"; }
        else if (hue < 350){shootBall = "Purple"; }
        else {shootBall = "None"; }
        frontSlot = frontDistance.getDistance(DistanceUnit.CM) < 8;

    }
    public void updateRGB(){
        if (!(rightFly.getVelocity()<1180)){RGB2.setPosition(.5);}
        else{RGB2.setPosition(.28);}
        switch (shootBall) {
            case "Green": RGB1.setPosition(.5);break;
            case "Purple": RGB1.setPosition(.72);break;
            case "None": RGB1.setPosition(0);break;
        }
    }
}
