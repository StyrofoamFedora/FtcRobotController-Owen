package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.CRServo;

public class Robot {

    private static final String TAG = "Robot"; // Tag for logging

    public DcMotor leftFrontDrive;
    public DcMotor leftBackDrive;
    public DcMotor rightFrontDrive;
    public DcMotor rightBackDrive;
    public DcMotorEx rightFly;
    public DcMotorEx spindexer;
    public DcMotor Intake;
    public CRServo lift;
    public Servo RGB1;
    public Servo RGB2;
    public RevColorSensorV3 shootColor;
    public DistanceSensor frontDistance;
    public IMU imu;

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
            Intake = hardwareMap.get(DcMotor.class, "bottomIntake");
            spindexer = hardwareMap.get(DcMotorEx.class, "topIntake");
            imu = hardwareMap.get(IMU.class, "imu");
            // Adjust the orientation parameters to match your robot
            IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.UP,
                    RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
            imu.initialize(parameters);
            lift = hardwareMap.get(CRServo.class, "lift");
            RGB1 = hardwareMap.servo.get("rgb");
            RGB2 = hardwareMap.servo.get("rgb2");
            frontDistance = hardwareMap.get(DistanceSensor.class,"frontSlot");
            rightFly.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            spindexer.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            spindexer.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            spindexer.setTargetPositionTolerance(2);
            spindexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            PIDCoefficients pidCoefficientsSpin = new PIDCoefficients(22,0,11);
            spindexer.setPIDCoefficients(DcMotorEx.RunMode.RUN_TO_POSITION,pidCoefficientsSpin);
            leftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
            leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);
            //rightFly.setDirection(DcMotorSimple.Direction.REVERSE);
        } catch (Exception e) {
            Log.e(TAG, "Error initializing hardware", e); // Replaces e.printStackTrace()
        }
    }
    public void updateDriveMotors(double x, double y, double rx) {
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
        rotX = rotX * 1.1;  // Counteract imperfect strafing
        // Normalize the values so no wheel power exceeds 100%
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double leftFrontPower = (rotY + rotX + rx) / denominator;
        double leftBackPower = (rotY - rotX + rx) / denominator;
        double rightFrontPower = (rotY - rotX - rx) / denominator;
        double rightBackPower = (rotY + rotX - rx) / denominator;
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }
    public void updateFlywheelMotors(double Velocity) {
        double F = 0.007*(Velocity+4.8);
        double P = 80;
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P,0,0,F);
        rightFly.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        rightFly.setVelocity(Velocity);
    }
    public void updateIntakeMotors(double powerIntake) {
        Intake.setPower(1 * powerIntake);
    }
    public void updateSpindexSlot(double slots, double powerSpin){
        double CPR = 537.7;
        double slotTicks = CPR/3;
        currentTick += (slotTicks*slots);
        int currentIntTick = (int)Math.round(currentTick);
            spindexer.setTargetPosition(currentIntTick);
            spindexer.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            spindexer.setPower(powerSpin);
    }
    public void updateColorDistance(){
//        double hue = JavaUtil.colorToHue(shootColor.getNormalizedColors().toColor());
//        if(hue < 80){shootBall = "None"; }
//        else if (hue < 200) {shootBall = "Green"; }
//        else if (hue < 350){shootBall = "Purple"; }
//        else {shootBall = "None"; }
        frontSlot = frontDistance.getDistance(DistanceUnit.CM) < 5;
    }
    public void updateRGB(){
        if (frontDistance.getDistance(DistanceUnit.CM) < 5){RGB2.setPosition(.5);}
        else{RGB2.setPosition(.28);}
    }
    public void updateLift(double lifterPower){
        lift.setPower(lifterPower);
    }
}