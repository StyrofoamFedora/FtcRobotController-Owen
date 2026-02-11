package org.firstinspires.ftc.teamcode.tele;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Webcam;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Config
@TeleOp(name = "New Tele-op", group = "Driver Op Mode")
public class TESTINGMecanumTeleOp extends LinearOpMode {
    private Robot robot;
    boolean isTargetLock = false;
    double axial = 0;
    double lateral = 0;
    double yaw = 0;
    private double flywheelControl = 0;
    double intakeControl = 0;
    private final Webcam aprilTagWebcam = new Webcam();

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
            updateKick();
            robot.updateRGB();
            robot.updateColorDistance();
        }
    }
    private void updateDrive(){
        isTargetLock = !gamepad1.right_stick_button;
        double error;
        double goalX = 0;
        if (isTargetLock){
            aprilTagWebcam.update();
            AprilTagDetection id20 = aprilTagWebcam.getTagBySpecificId(20);
           if (id20 != null){
               error = goalX - id20.ftcPose.bearing;
               if (Math.abs(error) < 0.4){
                   yaw = 0;
                }else{
                   yaw = Range.clip((error*0.005),-0.4,0.4);
                }
           }else {yaw = (1.0 * gamepad1.right_stick_x);}
        }else {yaw = (1.0 * gamepad1.right_stick_x);} // Rotate
        axial = (1.0 * gamepad1.left_stick_y); // FWD/REV
        lateral = (0.8 * (gamepad1.left_stick_x)); // Strafing
        robot.updateDriveMotors(axial, lateral, yaw);
    }
    private void updateFlywheel(){
       double goalDist;
        if(isTargetLock) {
            aprilTagWebcam.update();
            AprilTagDetection id20 = aprilTagWebcam.getTagBySpecificId(20);
            goalDist = Math.sqrt(Math.pow(id20.ftcPose.x,2)+Math.pow(id20.ftcPose.y,2));
            flywheelControl = 1072.45*Math.pow(1.00332,goalDist);
        }else{
            if (gamepad2.left_bumper) {flywheelControl = 1200;}
            else if (gamepad2.a) {flywheelControl = 0;}
            if (gamepad2.dpadUpWasPressed()) {flywheelControl += 20;}
            if (gamepad2.dpadDownWasPressed()) {flywheelControl -= 20;}
        }
        robot.updateFlywheelMotors(flywheelControl);
    }
    private void updateIntake(){
        if (gamepad1.left_trigger>0.2){intakeControl = -0.8;}
        else if (gamepad1.right_trigger>0.2) {intakeControl=0.8;}
        else {intakeControl=0;}
        robot.updateIntakeMotors(intakeControl);
    }
    private void updateSpindex(){
        if(gamepad2.dpadLeftWasPressed()){robot.updateSpindexSlot(1,0.3);}
        else if (gamepad2.dpadRightWasPressed()){robot.updateSpindexSlot(-1,0.3);}
        else if(gamepad2.xWasPressed()){robot.updateSpindexSlot(0.05,0.3);}
        else if (gamepad2.yWasPressed()){robot.updateSpindexSlot(0.05,0.3);}
    }
    private void updateKick(){
        if(gamepad2.right_trigger>0.2){robot.updateKickServo(0);}
        else{robot.updateKickServo(1);}
    }
}