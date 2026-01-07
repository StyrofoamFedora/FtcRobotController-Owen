package org.firstinspires.ftc.teamcode.tele;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class NewFieldCentricMecanumTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        // Make sure your ID's match your configuration
         DcMotor backLeftMotor = hardwareMap.get(DcMotor.class,"left_back_drive");
         DcMotor frontRightMotor = hardwareMap.get(DcMotor.class, "right_front_drive");
         DcMotor backRightMotor = hardwareMap.get(DcMotor.class,"right_back_drive");
         DcMotor frontLeftMotor = hardwareMap.get(DcMotor.class,"left_front_drive");
         DcMotorEx rightFly = hardwareMap.get(DcMotorEx.class,"rightFly");
         //DcMotorEx leftFly = hardwareMap.get(DcMotorEx.class,"leftFly");
         DcMotorEx spindexer = hardwareMap.get(DcMotorEx.class,"topIntake");
         DcMotor Intake = hardwareMap.get(DcMotor.class, "bottomIntake");
         Servo kick = hardwareMap.servo.get("ethan");
        Servo RGB = hardwareMap.servo.get("rgb");
        //leftFly.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightFly.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        spindexer.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        spindexer.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        spindexer.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        spindexer.setTargetPositionTolerance(3);
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

        //Gamepad Setups for Rising Edge Detectors
        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

        //Set Values for PID and Intake
        double targetFlywheelVelo = 0;
        double Rintegral = 0;
        double Lintegral = 0;
        double previousLFError = 0;
        double previousRFError = 0;
        double Kp = 0;
        double Ki = 0;
        double kd = 0;
        double intakePower = 0;
        int ballSlot = 0;
        int CPR = 538;
        int slotTicks = CPR/3;
        int slotOne = 1;
        int slotTwo = 1;
        int slotThree = 2;
        double spindexTarget = 0;

        waitForStart();

        if (isStopRequested()) return;

        //Tele-op Loop
        while (opModeIsActive()) {

            //Set up game pads
            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            if (gamepad1.optionsWasPressed()) {
                imu.resetYaw();
            }
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
            if (currentGamepad1.left_trigger>0.2 && previousGamepad1.left_trigger<0.2){intakePower=-0.8;}//Outtake
            if (currentGamepad1.right_trigger<0.2 && previousGamepad1.right_trigger>0.2){intakePower=0;}//Intake Shutoff
            if (currentGamepad1.left_trigger<0.2 && previousGamepad1.left_trigger>0.2){intakePower=0;}//Outtake Shutoff
            if (currentGamepad2.dpad_left && !previousGamepad2.dpad_left){spindexer.setPower(.2);}
            if (currentGamepad2.dpad_right && !previousGamepad2.dpad_right){spindexer.setPower(-.2);}
            // Match Buttons Pressed Gamepad 2 (Shooter[ball slots])
            if (currentGamepad2.x && !previousGamepad2.x){
                if(slotOne == 2){
                    spindexer.setTargetPosition(spindexer.getCurrentPosition()+(slotTicks*(ballSlot-1)));
                }else if(slotTwo == 2){
                    spindexer.setTargetPosition(spindexer.getCurrentPosition()+(slotTicks*(ballSlot-2)));
                }
                else if(slotThree == 2){
                    spindexer.setTargetPosition(spindexer.getCurrentPosition()+(slotTicks*(ballSlot-3)));
                }
            }//go to next Purple ball
            if (currentGamepad2.a && !previousGamepad2.a){
                if(slotOne == 1){
                    spindexer.setTargetPosition(spindexer.getCurrentPosition()+(slotTicks*(ballSlot-1)));}
                else if(slotTwo == 1){
                    spindexer.setTargetPosition(spindexer.getCurrentPosition()+(slotTicks*(ballSlot-2)));}
                else if(slotThree == 1){
                    spindexer.setTargetPosition(spindexer.getCurrentPosition()+(slotTicks*(ballSlot-3)));}

            }//go to next green ball
            if (currentGamepad2.dpad_left && !previousGamepad2.dpad_left && !spindexer.isBusy()){
                spindexer.setTargetPosition(spindexer.getCurrentPosition()-slotTicks);
                spindexer.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                spindexer.setPower(.25);
                ballSlot -= 1;
            } //Previous Ball Slot
            if (currentGamepad2.dpad_right && !previousGamepad2.dpad_right && !spindexer.isBusy()){
                spindexer.setTargetPosition(spindexer.getCurrentPosition()+slotTicks);
                spindexer.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                spindexer.setPower(.25);
                ballSlot += 1;
            } //Next Ball Slot
            if (currentGamepad2.y && !previousGamepad2.y){if(ballSlot == 0){slotOne = 1;}else if(ballSlot == 1){slotTwo = 1;}else if(ballSlot == 2){slotThree = 1;}}//Set Current slot Green
            if (currentGamepad2.b && !previousGamepad2.b){if(ballSlot == 0){slotOne = 2;}else if(ballSlot == 1){slotTwo = 2;}else if(ballSlot == 2){slotThree = 2;}}//Set Current slot Purple
            if (currentGamepad2.left_stick_x>0.2 && previousGamepad2.left_stick_x<0.2){spindexer.setPower(.25);}
            if (currentGamepad2.left_stick_x<-0.2 && previousGamepad2.left_stick_x>-0.2){spindexer.setPower(-.25);}
            //Rest of the Gamepad2 controls {Shooter[shooting]}
            if (currentGamepad2.right_trigger>0.2 && previousGamepad2.right_trigger<0.2){kick.setPosition(0);}// Shoot
            if (currentGamepad2.right_trigger<0.2 && previousGamepad2.right_trigger>0.2){kick.setPosition(1);}// UnSHoot
            if (currentGamepad2.left_trigger>0.2 && previousGamepad2.left_trigger<0.2){targetFlywheelVelo = 0.60;}//Spin Up
            if (currentGamepad2.left_bumper && !previousGamepad2.left_bumper){ targetFlywheelVelo = 0;} //Spin Down
            if (currentGamepad2.dpad_up && !previousGamepad2.dpad_up){targetFlywheelVelo += .02;} //Increase Power
            if (currentGamepad2.dpad_down && !previousGamepad2.dpad_down){targetFlywheelVelo -= .02;} //Decrease Power

            //PID Loop for motor velocity
            //double LFError = targetFlywheelVelo - leftFly.getVelocity();
            double RFError = targetFlywheelVelo - rightFly.getVelocity();
            //double leftFlywheelPower = targetFlywheelVelo+ (Kp*LFError) + (Ki*Lintegral) + (kd*(LFError-previousLFError));
            double rightFlywheelPower = targetFlywheelVelo +(Kp*RFError)+(Ki*Rintegral)+(kd*(RFError-previousRFError));
            //if (Math.abs(leftFlywheelPower)<.1){leftFlywheelPower=0;}
            if (Math.abs(rightFlywheelPower)<.1){rightFlywheelPower=0;}
            if (Math.abs(rightFlywheelPower)>rightFly.getVelocity()){RGB.setPosition(.28);}//Set LED to RED
            else{RGB.setPosition(.5);} //Set LED to GREEN

            //Run everything
            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);
            //leftFly.setPower(leftFlywheelPower);
            rightFly.setPower(-rightFlywheelPower);
            Intake.setPower(intakePower);


            //PID Loop
            //previousLFError = LFError;
            previousRFError = RFError;
            //Lintegral += LFError;
            Rintegral += RFError;
            ballSlot = Math.abs(ballSlot%3);

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