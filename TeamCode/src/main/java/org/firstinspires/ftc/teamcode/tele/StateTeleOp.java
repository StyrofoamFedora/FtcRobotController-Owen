package org.firstinspires.ftc.teamcode.tele;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Webcam;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Config
@TeleOp(name = "State Tele-op", group = "Driver Op Mode")
public class StateTeleOp extends LinearOpMode {
    private Robot robot;
    boolean isTargetLock = false;
    double xDrive,yDrive,rxDrive = 0;
    private double flywheelControl = 0;
    double intakeControl = 0;
    double liftControl = 0;
    double spindexSlots = 0;
    double spindexPowerControl = 0.4;
    double error;
    private final Webcam aprilTagWebcam = new Webcam();
    boolean frontSlot;

    @Override
    public void runOpMode(){
        robot = new Robot();
        robot.initialize((hardwareMap));
        aprilTagWebcam.init(hardwareMap,telemetry);
        waitForStart();
        while (opModeIsActive()){
            updateDrive();
            updateFlywheel();
            updateSpindex();
            updateIntake();
            updateLift();
            updateRGB();
            telemetry.addData("target Lock", isTargetLock);
            telemetry.addData("Fly", flywheelControl);
            telemetry.addData("error",error);
            telemetry.addData("intake", intakeControl);
            telemetry.addData("lift", liftControl);
            telemetry.addData("spindex",spindexPowerControl);
            telemetry.addData("spindex",spindexSlots);
            telemetry.update();
        }
    }
    private void updateDrive(){
        if (gamepad1.options) {robot.imu.resetYaw();}
        isTargetLock = gamepad1.right_stick_button;
        double goalX = 0;
        double rotationReduction = 0.75;
        if (isTargetLock){
            aprilTagWebcam.update();
            AprilTagDetection id20 = aprilTagWebcam.getTagBySpecificId(20);
            AprilTagDetection id19 = aprilTagWebcam.getTagBySpecificId(19);
           if (id20 != null){
               error = goalX - id20.ftcPose.bearing;
               if (Math.abs(error) < 0.4){
                   yDrive = 0;
                }else{yDrive = -Range.clip((error*0.05),-0.4,0.4);}
           }
           if (id19 != null) {
               error = goalX - id19.ftcPose.bearing;
               if (Math.abs(error) < 0.4) {
                   yDrive = 0;
               }else{yDrive = -Range.clip((error * 0.05), -0.4, 0.4);}
           }
        }else {rxDrive = (-rotationReduction * gamepad1.right_stick_x);} // Rotate
        yDrive = (1 * gamepad1.left_stick_y); // FWD/REV
        xDrive = (-gamepad1.left_stick_x); // Strafing
        robot.updateDriveMotors(xDrive, yDrive, rxDrive);
    }
    private void updateFlywheel(){
       double goalDist;
        if(isTargetLock) {
            aprilTagWebcam.update();
            AprilTagDetection id20 = aprilTagWebcam.getTagBySpecificId(20);
            if (id20 != null) {
                goalDist = Math.sqrt(Math.pow(id20.ftcPose.x, 2) + Math.pow(id20.ftcPose.y, 2));
                flywheelControl = 1072.45 * Math.pow(1.00332, goalDist);
            }
        }else{
            if (gamepad2.leftBumperWasPressed()) {flywheelControl = 1300;}
            else if (gamepad2.bWasPressed()) {flywheelControl = 0;}
            if (gamepad2.dpadUpWasPressed()) {flywheelControl += 20;}
            if (gamepad2.dpadDownWasPressed()) {flywheelControl -= 20;}
        }
        robot.updateFlywheelMotors(flywheelControl);
    }
    private void updateIntake(){
        if (gamepad1.left_trigger>0.2){intakeControl = -0.8;}
        else if (gamepad1.right_trigger>0.2) {intakeControl=0.8;}
        else if (gamepad2.a){intakeControl = 0.8;}
        else {intakeControl=0;}
        robot.updateIntakeMotors(intakeControl);
    }
    private void updateSpindex(){
        if(gamepad2.dpadRightWasPressed()){spindexSlots = -1;}
        else if(gamepad2.xWasPressed()){spindexSlots = -0.1;}
        else if (gamepad2.yWasPressed()){spindexSlots = 0.1;}
        else if(gamepad2.aWasPressed() && robot.rightFly.getVelocity()>1000){spindexPowerControl = 0.6; spindexSlots = 4;}
        robot.updateSpindexSlot(spindexSlots,spindexPowerControl);
        spindexPowerControl = 0.4;
        spindexSlots = 0;
    }
    private void updateLift(){
        if(gamepad2.options){liftControl = 0.5;}
        else if (gamepad2.right_stick_button){liftControl = -0.5;}
        else{liftControl = 0;}
        robot.updateLift(liftControl);
    }
    private void updateDistance(){
        robot.updateColorDistance();
        frontSlot = robot.frontSlot;
    }
    private void updateRGB(){
        robot.updateRGB();
    }
}