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
@Autonomous(name = "BFM5", group = "Autonomous")
public class BlueFarStateM5 extends LinearOpMode {
    //Create Default Variables for Vision Sets
    int apriltagid = 21;
    //Set up Classes for Trajectory + Actions
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(60, -16, Math.toRadians(210));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        TestingAutoRobot.shooter shooter = new TestingAutoRobot.shooter(hardwareMap);
        TestingAutoRobot.combine combine = new TestingAutoRobot.combine(hardwareMap);
        TestingAutoRobot.spindex spindex = new TestingAutoRobot.spindex(hardwareMap);
        TestingAutoRobot.Eyes eyes = new TestingAutoRobot.Eyes(hardwareMap);
        TestingAutoRobot.intakeLock intakeLock = new TestingAutoRobot.intakeLock(hardwareMap);
        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.addData("Position during Init", apriltagid);
            telemetry.update();
            if (isStopRequested()) return;
        }
        waitForStart();
// Trajectories

        TrajectoryActionBuilder intakeBottomSet = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(36,-20),Math.toRadians(270));
        TrajectoryActionBuilder intakingBottom = drive.actionBuilder(new Pose2d(36,-20,Math.toRadians(270)))
                .strafeTo(new Vector2d(36,-32))
                .strafeTo(new Vector2d(36,-55), new TranslationalVelConstraint(4));
        TrajectoryActionBuilder shootSet2 = drive.actionBuilder(new Pose2d(36,-55,Math.toRadians(270)))
                .strafeToLinearHeading(new Vector2d(60,-16), Math.toRadians(210));
        TrajectoryActionBuilder intakeMiddleSet = drive.actionBuilder(new Pose2d(-30,-20,Math.toRadians(231.5)))
                .strafeToLinearHeading(new Vector2d(12,-20),Math.toRadians(270));
        TrajectoryActionBuilder intakingMiddle = drive.actionBuilder(new Pose2d(12,-20,Math.toRadians(270)))
                .strafeTo(new Vector2d(12,-30))
                .strafeTo(new Vector2d(12,-50), new TranslationalVelConstraint(4));
        TrajectoryActionBuilder shootSet3 = drive.actionBuilder(new Pose2d(12,-50,Math.toRadians(270)))
                .strafeToLinearHeading(new Vector2d(-30,-20), Math.toRadians(233.5));
        TrajectoryActionBuilder outsideSet = drive.actionBuilder(new Pose2d(60,-16,Math.toRadians(233.5)))
                .strafeTo(new Vector2d(65,-55));

//Actions Part 2 Electric Boogaloo
        Action waitForTag = new TestingAutoRobot.WaitForTagAction(eyes,1500);
        Action autoIntake = new SequentialAction(spindex.waitForBall(),spindex.prevSlot());
//Vision Set + Looking

//Remaining Driving and shooting
        Actions.runBlocking(new SequentialAction(

                shooter.spinUpFar(),
                new SleepAction(1.25),
                intakeLock.unlock(),
                combine.intake(),
                spindex.unload(),
                new SleepAction(1.5),
                outsideSet.build(),
                new SleepAction(1.5)
        ));
    }
}