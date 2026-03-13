package org.firstinspires.ftc.teamcode.tele;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Webcam;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Config
@TeleOp(name = "Solo State Tele-op", group = "Driver Op Mode")
public class SoloStateTeleOp extends LinearOpMode {
    private Robot robot;
    boolean isTargetLock = false;
    double xDrive,yDrive,rxDrive = 0;
    private double flywheelControl = 0;
    double intakeControl = 0;
    double liftControl = 0;
    double spindexSlots = 0;
    double spindexPowerControl = 0.4;
    double intakeLockControl;
    double camError;
    private final Webcam aprilTagWebcam = new Webcam();
    boolean frontSlot, blSlot, ballStorage1, ballStorage3 = false;
    ElapsedTime autoIntakeTimer = new ElapsedTime();
    ElapsedTime multiSee = new ElapsedTime();

    @Override
    public void runOpMode(){
        robot = new Robot();
        robot.initialize((hardwareMap));
        aprilTagWebcam.init(hardwareMap,telemetry);
        waitForStart();
        while (opModeIsActive()){
            updateDistance();
            updateDrive();
            updateFlywheel();
            updateSpindex();
            updateIntake();
            updateLift();
            updateRGB();
            telemetry.addData("target Lock", isTargetLock);
            telemetry.addData("Fly", flywheelControl);
            telemetry.addData("error", camError);
            telemetry.addData("intake", intakeControl);
            telemetry.addData("lift", liftControl);
            telemetry.addData("spindex",spindexPowerControl);
            telemetry.addData("spindex",spindexSlots);
            telemetry.addData("lock", intakeLockControl);
            telemetry.addData("blSlot", blSlot);
            telemetry.addData("fSlot", frontSlot);
            telemetry.addData("ballStorage1", ballStorage1);
            telemetry.addData("ballStorage3", ballStorage3);
            telemetry.addData("timerAutoIntake", autoIntakeTimer.seconds());
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
               camError = goalX - id20.ftcPose.bearing;
               if (Math.abs(camError) < 0.4){
                   rxDrive = 0;
                }else{rxDrive = -Range.clip((camError *0.07),-0.4,0.4);}
           }
           if (id19 != null) {
               camError = goalX - id19.ftcPose.bearing;
               if (Math.abs(camError) < 0.4) {
                   rxDrive = 0;
               }else{rxDrive = -Range.clip((camError * 0.07), -0.4, 0.4);}
           }
        }else {rxDrive = (rotationReduction * gamepad1.right_stick_x);} // Rotate
        yDrive = (-1 * gamepad1.left_stick_y); // FWD/REV
        xDrive = (gamepad1.left_stick_x); // Strafing
        robot.updateDriveMotors(xDrive, yDrive, rxDrive);
    }
    private void updateFlywheel(){
       double goalDist;
        if(isTargetLock) {
            aprilTagWebcam.update();
            AprilTagDetection id20 = aprilTagWebcam.getTagBySpecificId(20);
            if (id20 != null) {
                goalDist = id20.ftcPose.range;
            //Math.sqrt(Math.pow(id20.ftcPose.x, 2) + Math.pow(id20.ftcPose.y, 2));
                flywheelControl = 1072.45 * Math.pow(1.00332, goalDist);
            }
        }else{
            if (gamepad1.leftBumperWasPressed()) {flywheelControl = 1240;}
            else if (gamepad1.bWasPressed()) {flywheelControl = 1600;}
            if (gamepad1.dpadUpWasPressed()) {flywheelControl += 20;}
            if (gamepad1.dpadDownWasPressed()) {flywheelControl -= 20;}
        }
        robot.updateFlywheelMotors(flywheelControl);
    }
    private void updateIntake(){
        if (gamepad1.left_trigger>0.2){intakeControl = -0.8;intakeLockControl=0.9;}
        else if (gamepad1.right_trigger>0.2) {intakeControl=0.8;intakeLockControl=0.9;ballStorage1 = false;}
        else if (gamepad1.a){intakeControl = 0.8;intakeLockControl=0;}
        else {intakeControl=0;intakeLockControl =0.45;}
        robot.updateIntakeMotors(intakeControl,intakeLockControl);
    }
    private void updateSpindex(){
        if(frontSlot && !robot.spindexer.isBusy() && autoIntakeTimer.seconds()>1.0 && !ballStorage3){
            if(multiSee.seconds()>0.5) {
                if (blSlot) {
                    intakeControl = 0;
                    intakeLockControl = 0.4;
                    ballStorage3 = true;
                    spindexSlots = -1;
                } else {
                    spindexSlots = -1;
                    ballStorage1 = true;
                    autoIntakeTimer.reset();
                }
            }
            else{multiSee.reset();}
        }
        else {
            if (gamepad1.dpadRightWasPressed()) {spindexSlots = -1;}
            else if (gamepad1.xWasPressed()) {spindexSlots = -0.1;}
            else if (gamepad1.yWasPressed()) {spindexSlots = 0.1;}
            else if (gamepad1.aWasPressed() && robot.rightFly.getVelocity() > 1000) {
                spindexPowerControl = 0.6;
                spindexSlots = 4;
                ballStorage3 = false;
                ballStorage1 = false;
            }
        }
        robot.updateSpindexSlot(spindexSlots, spindexPowerControl);
        spindexPowerControl = 0.4;
        spindexSlots = 0;
    }
    private void updateLift(){
        if(gamepad1.share){liftControl = 0.5;}
        else if (gamepad1.left_stick_button){liftControl = -0.5;}
        else{liftControl = 0;}
        robot.updateLift(liftControl);
    }
    private void updateDistance(){
        robot.updateColorDistance();
        frontSlot = robot.frontSlot;
        blSlot = robot.blSlot;
    }
    private void updateRGB(){
        robot.updateRGB();
    }
}