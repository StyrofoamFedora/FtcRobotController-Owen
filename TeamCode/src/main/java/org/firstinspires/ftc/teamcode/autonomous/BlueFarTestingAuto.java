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
@Autonomous(name = "NBF_AUTO_TEST", group = "Autonomous")
public class BlueFarTestingAuto extends LinearOpMode {
    //Create Default Variables for Vision Sets
    int apriltagid = 21;
    //Set up Classes for Trajectory + Actions
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(-56, -46, Math.toRadians(235.5));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        TestingAutoRobot.shooter shooter = new TestingAutoRobot.shooter(hardwareMap);
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
                .strafeToLinearHeading(new Vector2d(-40,-25),Math.toRadians(170));
        TrajectoryActionBuilder shootSet1 = drive.actionBuilder(new Pose2d(-40,-25,Math.toRadians(170)))
                .strafeToLinearHeading(new Vector2d(-30,-20),Math.toRadians(231.5));
        TrajectoryActionBuilder intakeTopSet = drive.actionBuilder(new Pose2d(-30,-20,Math.toRadians(231.5)))
                .strafeToLinearHeading(new Vector2d(-11,-20),Math.toRadians(270));
        TrajectoryActionBuilder intakingTop = drive.actionBuilder(new Pose2d(-11,-20,Math.toRadians(270)))
                .strafeTo(new Vector2d(-11,-30))
                .strafeTo(new Vector2d(-11,-45), new TranslationalVelConstraint(4));
        TrajectoryActionBuilder shootSet2 = drive.actionBuilder(new Pose2d(-11,-45,Math.toRadians(270)))
                .strafeToLinearHeading(new Vector2d(-30,-20), Math.toRadians(231.5));
        TrajectoryActionBuilder intakeMiddleSet = drive.actionBuilder(new Pose2d(-30,-20,Math.toRadians(231.5)))
                .strafeToLinearHeading(new Vector2d(12,-20),Math.toRadians(270));
        TrajectoryActionBuilder intakingMiddle = drive.actionBuilder(new Pose2d(12,-20,Math.toRadians(270)))
                .strafeTo(new Vector2d(12,-30))
                .strafeTo(new Vector2d(12,-45), new TranslationalVelConstraint(4));
        TrajectoryActionBuilder shootSet3 = drive.actionBuilder(new Pose2d(12,-45,Math.toRadians(270)))
                .strafeToLinearHeading(new Vector2d(-30,-20), Math.toRadians(233.5));
        TrajectoryActionBuilder outsideSet = drive.actionBuilder(new Pose2d(-30,-20,Math.toRadians(233.5)))
                .strafeTo(new Vector2d(0,-40));

//Actions Part 2 Electric Boogaloo
        Action waitForTag = new TestingAutoRobot.WaitForTagAction(eyes,1500);
        Action autoIntake = new SequentialAction(spindex.waitForBall(),spindex.prevSlot());
//Vision Set + Looking
        Actions.runBlocking(new SequentialAction(
                visionSet.build(),
                waitForTag
        ));
//ball organizing Actions
        Action ballOrganize;
        Action ballOrganize2;
        if (eyes.detectedTag==21){
            ballOrganize = spindex.unload();
            ballOrganize2 = spindex.unload();
        } else if (eyes.detectedTag==23) {
            ballOrganize = spindex.prevSlot();
            ballOrganize2 = spindex.prevSlot();
        } else  {
            ballOrganize = new SleepAction(.01);
            ballOrganize2 = new SleepAction(.01);
        }
//Remaining Driving and shooting
        Actions.runBlocking(new SequentialAction(
                ballOrganize,
                shooter.spinUpClose(),
                shootSet1.build(),
                combine.intake(),
                spindex.unload(),
                new SleepAction(0.5),
                intakeTopSet.build(),
                new ParallelAction(
                        intakingTop.build(),
                        new SequentialAction(
                                autoIntake,
                                autoIntake,
                                spindex.waitForBall()
                        )
                ),
                ballOrganize2,
                combine.outtake(),
                new SleepAction(.5),
                combine.holdtake(),
                shootSet2.build(),
                combine.intake(),
                spindex.unload(),
                new SleepAction(0.5),
                intakeMiddleSet.build(),
                combine.intake(),
                new ParallelAction(
                        intakingMiddle.build(),
                        new SequentialAction(
                                autoIntake,
                                autoIntake,
                                spindex.waitForBall()
                        )
                ),
                shootSet3.build(),
                combine.intake(),
                spindex.unload(),
                new SleepAction(0.5),
                outsideSet.build()
        ));
    }
}