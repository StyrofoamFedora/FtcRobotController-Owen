package org.firstinspires.ftc.teamcode;
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
public class FieldCentricMecanumTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        // Make sure your ID's match your configuration
         DcMotor backLeftMotor = hardwareMap.get(DcMotor.class,"left_back_drive");
         DcMotor frontRightMotor = hardwareMap.get(DcMotor.class, "right_front_drive");
         DcMotor backRightMotor = hardwareMap.get(DcMotor.class,"right_back_drive");
         DcMotor frontLeftMotor = hardwareMap.get(DcMotor.class,"left_front_drive");
         DcMotorEx rightFly = hardwareMap.get(DcMotorEx.class,"rightFly");
         DcMotorEx leftFly = hardwareMap.get(DcMotorEx.class,"leftFly");
         DcMotor topIntake = hardwareMap.get(DcMotor.class,"topIntake");
         DcMotor bottomIntake = hardwareMap.get(DcMotor.class, "bottomIntake");
         Servo kick = hardwareMap.servo.get("ethan");
        leftFly.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightFly.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // CHECK IF NEEDED, FOR IF MOTORS ARE BACKWARDS
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
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
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;
            // Reset Yaw to zero when the start button is pressed
            if (gamepad1.options) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // rationalize the motor power to be less than 1
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            //Match Buttons Pressed
            if (currentGamepad1.right_trigger>0.2 && previousGamepad1.right_trigger<0.2){intakePower = 0.8;}//Intake
            if (currentGamepad1.left_trigger>0.2 && previousGamepad1.left_trigger<0.2){intakePower=-0.8;}//Outtake
            if (currentGamepad1.right_trigger<0.2 && previousGamepad1.right_trigger>0.2){intakePower=0;}//Intake Shutoff
            if (currentGamepad1.left_trigger<0.2 && previousGamepad1.left_trigger>0.2){intakePower=0;}//Outtake Shutoff
            if (currentGamepad2.a && !previousGamepad2.a){targetFlywheelVelo = .60;}//Spin Up
            if (currentGamepad2.b && !previousGamepad2.b){targetFlywheelVelo = 0;}//Spin Down
            if (currentGamepad2.right_trigger>0.2 && previousGamepad2.right_trigger<0.2){kick.setPosition(0.5);}// Shoot
            if (currentGamepad2.right_trigger<0.2 && previousGamepad2.right_trigger>0.2){kick.setPosition(1);}// UnSHoot
            if (currentGamepad2.left_bumper && !previousGamepad2.left_bumper){ targetFlywheelVelo += .02;} //Increase Power
            if (currentGamepad2.right_bumper && !previousGamepad2.right_bumper){targetFlywheelVelo -=.02;} //Decrease Power

            //PID Loop for motor velocity
            double LFError = targetFlywheelVelo - leftFly.getVelocity();
            double RFError = targetFlywheelVelo - rightFly.getVelocity();
            double leftFlywheelPower = targetFlywheelVelo+ (Kp*LFError) + (Ki*Lintegral) + (kd*(LFError-previousLFError));
            double rightFlywheelPower = targetFlywheelVelo +(Kp*RFError)+(Ki*Rintegral)+(kd*(RFError-previousRFError));
            if (Math.abs(leftFlywheelPower)<.1){leftFlywheelPower=0;}
            if (Math.abs(rightFlywheelPower)<.1){rightFlywheelPower=0;}

            //Run everything
            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);
            leftFly.setPower(leftFlywheelPower);
            rightFly.setPower(-rightFlywheelPower);
            bottomIntake.setPower(intakePower);
            topIntake.setPower(intakePower);
            //kick.setPosition(0);

            //PID Loop
            previousLFError = LFError;
            previousRFError = RFError;
            Lintegral += LFError;
            Rintegral += RFError;

            //telemetry
            telemetry.addData("Flywheel %",targetFlywheelVelo*100);
            telemetry.update();
        }
    }
}