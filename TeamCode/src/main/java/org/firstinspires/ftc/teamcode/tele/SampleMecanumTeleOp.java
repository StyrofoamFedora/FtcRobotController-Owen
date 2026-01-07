package org.firstinspires.ftc.teamcode.tele;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class SampleMecanumTeleOp extends LinearOpMode {
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
        Gamepad currentGamepad2 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

        //Set Values for PID and Intake
        double targetFlywheelVelo = 0;
        double intakePower = 0;
        int ballSlot = 0;
        double CPR = 537.7;
        double slotTicks = CPR/3;
        int slotOne = 1;
        int slotTwo = 1;
        int slotThree = 2;
        int currentIntTick = 0;
        double currentTick = 0;
        waitForStart();

        if (isStopRequested()) return;

        //Tele-op Loop
        while (opModeIsActive()) {

            //Set up game pads
            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            //Take values from sticks
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = -gamepad1.left_stick_x*1.1;
            double rx = gamepad1.right_stick_x;
            // rationalize the motor power to be less than 1
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            //Match Buttons Pressed GamePad 1 (Driver [non-driving])

            //Run everything
            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);
            rightFly.setVelocity(-targetFlywheelVelo);
            Intake.setPower(intakePower);
            spindexer.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            spindexer.setPower(.3);

            //telemetry
            telemetry.addData("Flywheel %",targetFlywheelVelo*100);
            telemetry.addData("Current Slot",ballSlot);
            telemetry.addData("Slot1",slotOne);
            telemetry.addData("Slot2",slotTwo);
            telemetry.addData("Slot 3",slotThree);
            telemetry.update();
        }
    }
}