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
    int apriltagid = 21;
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
        TrajectoryActionBuilder shootSet1 = visionSet.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-30,-20),Math.toRadians(231.5));
        TrajectoryActionBuilder intakeTopSet = shootSet1.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-11,-20),Math.toRadians(270));
        TrajectoryActionBuilder intakingTop = intakeTopSet.endTrajectory().fresh()
                .strafeTo(new Vector2d(-11,-30))
                .strafeTo(new Vector2d(-11,-52), new TranslationalVelConstraint(4));
        TrajectoryActionBuilder shootSet2 = intakingTop.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-30,-20), Math.toRadians(231.5));
        TrajectoryActionBuilder outsideSet = shootSet2.endTrajectory().fresh()
                .strafeTo(new Vector2d(0,-40));

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
                shooter.spinUp(),
                visionSet.build(),
                waitForTag
        ));
        Action ballOrganize;
        Action ballOrganize2;
        if (eyes.detectedTag==21){
            ballOrganize = spindex.nextSlot();
            ballOrganize2 = spindex.nextSlot();
        } else if (eyes.detectedTag==23) {
            ballOrganize = spindex.prevSlot();
            ballOrganize2 = spindex.prevSlot();
        } else  {
            ballOrganize = new SleepAction(.1);
            ballOrganize2 = new SleepAction(.1);
        }
        Actions.runBlocking(new SequentialAction(
                ballOrganize,
                shootSet1.build(),
                unloadBalls,
                intakeTopSet.build(),
                shooter.spinDown(),
                combine.intake(),
                new ParallelAction(
                        intakingTop.build(),
                        new SequentialAction(
                                spindex.autoIntake(), spindex.nextSlot(),
                                spindex.autoIntake(), spindex.nextSlot(),
                                spindex.autoIntake()
                        )
                ),
                ballOrganize2,
                combine.holdtake(),
                combine.outtake(),
                shooter.spinUp(),
                new SleepAction(.5),
                combine.holdtake(),
                shootSet2.build(),
                new SequentialAction(
                        kick.kickUp(), new SleepAction(0.5),
                        kick.kickDown(), new SleepAction(0.3),
                        spindex.nextSlot(), new SleepAction(0.5),
                        kick.kickUp(), new SleepAction(0.5),
                        kick.kickDown(), new SleepAction(0.3),
                        spindex.nextSlot(), new SleepAction(0.5),
                        kick.kickUp(), new SleepAction(0.5),
                        kick.kickDown()),
                outsideSet.build()
        ));
    }
}
