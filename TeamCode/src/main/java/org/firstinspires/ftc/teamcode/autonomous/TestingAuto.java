package org.firstinspires.ftc.teamcode.autonomous;
//Imports

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.MecanumDrive;


//Class Define
@Config
@Autonomous(name = "AUTO_TEST", group = "Autonomous")
public class TestingAuto extends LinearOpMode {
    //Create Default Variables for Vision Sets
    int visionOutputPosition = 0;
    int apriltagid = 21;
    int CPR = 538;
    int slotTicks = CPR/3;


    //Set up Classes for Trajectory + Actions
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(-56, -46, Math.toRadians(235.5));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        TestingAutoRobot.shooter shooter = new TestingAutoRobot.shooter(hardwareMap);
        TestingAutoRobot.kick kick = new TestingAutoRobot.kick(hardwareMap);
        TestingAutoRobot.combine combine = new TestingAutoRobot.combine(hardwareMap);
        TestingAutoRobot.spindex spindex = new TestingAutoRobot.spindex(hardwareMap);
        TestingAutoRobot.Eyes eyes = new TestingAutoRobot.Eyes(hardwareMap);
        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.addData("Position during Init", apriltagid);
            telemetry.update();
            if (isStopRequested()) return;
        }
        waitForStart();
// Trajectories
        TrajectoryActionBuilder visionSet = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(-30,-20),Math.toRadians(180));
        TrajectoryActionBuilder shootSet1 = drive.actionBuilder(new Pose2d(-30,-20,Math.toRadians(180)))
                .strafeToLinearHeading(new Vector2d(-30,-20),Math.toRadians(231.5));
        TrajectoryActionBuilder intakeTopSet = drive.actionBuilder(new Pose2d(-30,-20, Math.toRadians(231.5)))
                .strafeToLinearHeading(new Vector2d(-11,-20),Math.toRadians(270));
        TrajectoryActionBuilder intakingTop = drive.actionBuilder(new Pose2d(-11,-20, Math.toRadians(270)))
                .strafeTo(new Vector2d(-11,-30))
                .strafeTo(new Vector2d(-11,-52), new TranslationalVelConstraint(4));
        TrajectoryActionBuilder outsideSet = drive.actionBuilder(new Pose2d(-30,-20,Math.toRadians(231.5)))
                .strafeTo(new Vector2d(0,-40));
        TrajectoryActionBuilder shootSet2 = drive.actionBuilder(new Pose2d(-11,-52,Math.toRadians(270)))
                .strafeToLinearHeading(new Vector2d(-30,-20), Math.toRadians(231.5));

        Action waitForTag = new TestingAutoRobot.WaitForTagAction(eyes,1500);
        Action unloadBalls =  new SequentialAction(
                kick.kickUp(), new SleepAction(0.2),
                kick.kickDown(), new SleepAction(0.5),
                spindex.nextSlot(), new SleepAction(1),
                kick.kickUp(), new SleepAction(0.2),
                kick.kickDown(), new SleepAction(1),
                spindex.nextSlot(), new SleepAction(1),
                kick.kickUp(), new SleepAction(0.2),
                kick.kickDown());

//Stuff That's run
        Actions.runBlocking(new SequentialAction(
                visionSet.build(),
                waitForTag
        ));
        Action ballOrganize;
        if (eyes.detectedTag==21){
            ballOrganize = spindex.nextSlot();
        } else if (eyes.detectedTag==23) {
            ballOrganize = spindex.prevSlot();
        } else  {
            ballOrganize = new SleepAction(1);
        }
        eyes.stop();
        Actions.runBlocking(new SequentialAction(
                new SleepAction(1),
                new ParallelAction(
                    ballOrganize,
                    shootSet1.build(),
                    shooter.spinUp()
                ),
                unloadBalls,
                shooter.spinDown()
        ));
    }
}
