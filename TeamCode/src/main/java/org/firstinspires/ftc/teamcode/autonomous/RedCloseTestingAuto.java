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
@Autonomous(name = "RBC_AUTO_TEST", group = "Autonomous")
public class RedCloseTestingAuto extends LinearOpMode {
    //Create Default Variables for Vision Sets
    int apriltagid = 1;
    //Set up Classes for Trajectory + Actions
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(-56, 46, Math.toRadians(130));
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
        TrajectoryActionBuilder visionSet = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(-40,25),Math.toRadians(200));
        TrajectoryActionBuilder shootSet1 = drive.actionBuilder(new Pose2d(-40,25,Math.toRadians(200)))
                .strafeToLinearHeading(new Vector2d(-30,20),Math.toRadians(133));
        TrajectoryActionBuilder intakeTopSet = drive.actionBuilder(new Pose2d(-30,20,Math.toRadians(133)))
                .strafeToLinearHeading(new Vector2d(-12,20),Math.toRadians(90));
        TrajectoryActionBuilder intakingTop = drive.actionBuilder(new Pose2d(-12,20,Math.toRadians(90)))
                .strafeTo(new Vector2d(-12,30))
                .strafeTo(new Vector2d(-12,45), new TranslationalVelConstraint(4));
        TrajectoryActionBuilder shootSet2 = drive.actionBuilder(new Pose2d(-12,45,Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(-30,20), Math.toRadians(133));
        TrajectoryActionBuilder intakeMiddleSet = drive.actionBuilder(new Pose2d(-30,20,Math.toRadians(133)))
                .strafeToLinearHeading(new Vector2d(14,20),Math.toRadians(90));
        TrajectoryActionBuilder intakingMiddle = drive.actionBuilder(new Pose2d(14,20,Math.toRadians(90)))
                .strafeTo(new Vector2d(14,30))
                .strafeTo(new Vector2d(14,45), new TranslationalVelConstraint(4));
        TrajectoryActionBuilder shootSet3 = drive.actionBuilder(new Pose2d(14,45,Math.toRadians(90)))
                .strafeTo(new Vector2d(15,30))
                .strafeToLinearHeading(new Vector2d(-30,20), Math.toRadians(133));
        TrajectoryActionBuilder outsideSet = drive.actionBuilder(new Pose2d(-30,20,Math.toRadians(133)))
                .strafeTo(new Vector2d(0,40));

//Actions Part 2 Electric Boogaloo
        Action waitForTag = new TestingAutoRobot.WaitForTagAction(eyes,500);
        Action autoIntake = new SequentialAction(spindex.waitForBall(), new SleepAction(0.2), spindex.prevSlot());
//Vision Set + Looking
        Actions.runBlocking(new SequentialAction(
                visionSet.build(),
                waitForTag
        ));
//ball organizing Actions
        Action ballOrganize;
        Action ballOrganize2;
        Action ballOrganize3;
        if (eyes.detectedTag==21){
            ballOrganize = spindex.prevSlot();
            ballOrganize2 = spindex.prevSlot();
            ballOrganize3 = spindex.prevSlot();
        } else if (eyes.detectedTag==22) {
            ballOrganize = new SequentialAction(spindex.prevSlot(),spindex.prevSlot());
            ballOrganize2 = new SequentialAction(spindex.prevSlot(),spindex.prevSlot());
            ballOrganize3 = new SequentialAction(spindex.prevSlot(),spindex.prevSlot());
        } else  {
            ballOrganize = new SleepAction(.01);
            ballOrganize2 = new SleepAction(.01);
            ballOrganize3 = new SleepAction(.01);
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
                intakeLock.lock(),
                new ParallelAction(
                        intakingTop.build(),
                        new SequentialAction(
                                spindex.waitForBall(), new SleepAction(0.7), spindex.prevSlot(),
                                spindex.waitForBall(), new SleepAction(0.7), spindex.prevSlot(),
                                spindex.waitForBall(), new SleepAction(0.7), combine.holdtake()
                        )
                ),
                intakeLock.unlock(),
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
                intakeLock.lock(),
                new ParallelAction(
                        intakingMiddle.build(),
                        new SequentialAction(
                                spindex.waitForBall(), new SleepAction(0.5), spindex.prevSlot(),
                                spindex.waitForBall(), new SleepAction(0.7), spindex.prevSlot(),
                                spindex.waitForBall(), new SleepAction(0.7), combine.holdtake()
                        )
                ),
                intakeLock.unlock(),
                ballOrganize3,
                shootSet3.build(),
                combine.intake(),
                spindex.unload(),
                new SleepAction(0.5),
                outsideSet.build()
        ));
    }
}