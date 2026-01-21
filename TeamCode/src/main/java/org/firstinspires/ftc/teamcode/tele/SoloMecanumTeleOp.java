package org.firstinspires.ftc.teamcode.tele;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;



import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class SoloMecanumTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        // Make sure your ID's match your configuration
         DcMotor backLeftMotor = hardwareMap.get(DcMotor.class,"left_back_drive");
         DcMotor frontRightMotor = hardwareMap.get(DcMotor.class, "right_front_drive");
         DcMotor backRightMotor = hardwareMap.get(DcMotor.class,"right_back_drive");
         DcMotor frontLeftMotor = hardwareMap.get(DcMotor.class,"left_front_drive");
         DcMotorEx rightFly = hardwareMap.get(DcMotorEx.class,"rightFly");
         DcMotorEx spindexer = hardwareMap.get(DcMotorEx.class,"topIntake");
         DcMotor Intake = hardwareMap.get(DcMotor.class, "bottomIntake");
         Servo kick = hardwareMap.servo.get("ethan");
         Servo RGB = hardwareMap.servo.get("rgb");
        ColorRangeSensor FrontColor = hardwareMap.get(ColorRangeSensor.class, "frontColor");
        rightFly.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        spindexer.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        spindexer.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        spindexer.setTargetPositionTolerance(2);
        spindexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spindexer.setTargetPosition(0);



        // CHECK IF NEEDED, FOR IF MOTORS ARE BACKWARDS
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
        imu.initialize(parameters);

        //Gamepad Setups for Rising Edge Detectors (just on triggers)
        Gamepad currentGamepad1 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();

        //Set Values for PID and Intake
        double targetFlywheelVelo = 0;
        double intakePower = 0;
        int ballSlot = 0;
        double P = 75;
        double F = 14;
        double CPR = 537.7;
        double slotTicks = CPR/3;
        int slotFront = 1;
        int slotLeft = 1;
        int slotRight = 2;
        int preSlotFront;
        int slotHold;
        int currentIntTick;
        double currentTick = 0;
        String frontBall;
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P,0,0,F);
        rightFly.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        waitForStart();

        if (isStopRequested()) return;

        //Tele-op Loop
        while (opModeIsActive()) {

            //Set up game pads
            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);

            //Read Color of Ball 1
            double hue = JavaUtil.colorToHue(FrontColor.getNormalizedColors().toColor());
            if(hue < 50){frontBall = "None"; preSlotFront = 0;}
            else if (hue < 150) {frontBall = "Green"; preSlotFront = 2;}
            else if (hue < 350){frontBall = "Purple"; preSlotFront = 1;}
            else {frontBall = "None"; preSlotFront = 0;}

            //Driving
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x*0.8;

            if (gamepad1.options) {imu.resetYaw();}
            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;


            //Match Buttons Pressed GamePad 1 (Driver [non-driving])
            if (currentGamepad1.right_trigger>0.2 && previousGamepad1.right_trigger<0.2){intakePower = 0.8;}//Intake
            if (currentGamepad1.rightBumperWasPressed()){intakePower=-0.8;}//Outtake
            if (currentGamepad1.right_trigger<0.2 && previousGamepad1.right_trigger>0.2){intakePower=0;}//Intake Shutoff
            if (gamepad1.rightBumperWasReleased()){intakePower=0;}//Outtake Shutoff
            if (gamepad1.dpadLeftWasPressed()){
                if (slotRight == 0){
                    slotLeft = slotRight;
                    slotRight = slotFront;
                    slotFront = preSlotFront;
                }
                else{
                    slotHold = slotRight;
                    slotLeft = slotRight;
                    slotRight = slotFront;
                    slotFront = slotHold;
                }
                ballSlot -= 1;
                currentTick-=slotTicks;
                currentIntTick = (int)Math.round(currentTick);
                spindexer.setTargetPosition(currentIntTick);
                spindexer.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                spindexer.setPower(.3);} //Previous Ball Slot
            if (gamepad1.dpadRightWasPressed()){
                if (slotRight == 0){
                    slotRight = slotLeft;
                    slotLeft = slotFront;
                    slotFront = preSlotFront;
                }
                else{
                    slotHold = slotRight;
                    slotRight = slotLeft;
                    slotLeft = slotFront;
                    slotFront = slotHold;
                }
                ballSlot += 1;
                currentTick-=slotTicks;
                currentIntTick = (int)Math.round(currentTick);
                spindexer.setTargetPosition(currentIntTick);
                spindexer.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                spindexer.setPower(.3);} //Next Ball Slot

            if (currentGamepad1.left_trigger>0.2 && previousGamepad1.left_trigger<0.2){kick.setPosition(0); slotLeft = 0;}// Shoot
            if (currentGamepad1.left_trigger<0.2 && previousGamepad1.left_trigger>0.2){kick.setPosition(1);}// UnShoot
            if (gamepad1.leftBumperWasPressed()){targetFlywheelVelo = 1400;}//Spin Up
            if (gamepad1.rightStickButtonWasPressed()){ targetFlywheelVelo = 0;} //Spin Down
            if (gamepad1.dpadUpWasPressed()){targetFlywheelVelo += 20;} //Increase Power
            if (gamepad1.dpadDownWasPressed()){targetFlywheelVelo -= 20;} //Decrease Power

            //COLOR OF LIGHT
            if (Math.abs(targetFlywheelVelo)>rightFly.getVelocity()){RGB.setPosition(.28);}//Set LED to RED
            else{RGB.setPosition(.5);} //Set LED to GREEN

            //Run Motors + Servos
            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);
            rightFly.setVelocity(-targetFlywheelVelo);
            Intake.setPower(intakePower);
            spindexer.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            spindexer.setPower(.3);

            ballSlot = Math.abs(ballSlot%3);

            //telemetry
            telemetry.addData("Flywheel %",targetFlywheelVelo);
            telemetry.addData("Current Slot",ballSlot);
            telemetry.addData("Slot1",slotFront);
            telemetry.addData("Slot2",slotLeft);
            telemetry.addData("Slot 3",slotRight);
            telemetry.addData("Front Ball Color", frontBall);
            telemetry.addData("hue", hue);
            telemetry.addData("Color", FrontColor.getNormalizedColors().toString());
            telemetry.addData("Color", FrontColor.getNormalizedColors().toColor());
            telemetry.addData("Red", FrontColor.getNormalizedColors().red);
            telemetry.addData("Blue", FrontColor.getNormalizedColors().blue);
            telemetry.addData("Green", FrontColor.getNormalizedColors().green);
            telemetry.update();
        }
    }
}