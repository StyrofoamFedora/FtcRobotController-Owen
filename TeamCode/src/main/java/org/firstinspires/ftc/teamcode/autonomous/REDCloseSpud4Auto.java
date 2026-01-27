package org.firstinspires.ftc.teamcode.autonomous;
//Imports

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

//Class Define
@Config
@Autonomous(name = "CloseRED", group = "Autonomous")
public class REDCloseSpud4Auto extends LinearOpMode {
    //Create Default Variables for Vision Sets
    int visionOutputPosition = 0;
    int CPR = 538;
    int slotTicks = CPR/3;

    //Set up Actions
    public class Eyes {
        public final AprilTagProcessor aprilTag;
        public final VisionPortal visionPortal;
        public int detectedTag = -1;
        public AprilTagDetection lastDetection = null;
        public Eyes(HardwareMap hardwareMap) {
            aprilTag = AprilTagProcessor.easyCreateWithDefaults();
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    hardwareMap.get(WebcamName.class, "eyes"), aprilTag
            );
        }
        /* Poll AprilTag detections */
        public void update() {
            List<AprilTagDetection> detections = aprilTag.getDetections();
            if (!detections.isEmpty()) {
                lastDetection = detections.get(0);
                detectedTag = lastDetection.id;
            }
        }

        /* Optional cleanup */
        public void stop() {
            visionPortal.close();
        }
    }
    public class WaitForTagAction implements Action {

        private final Eyes eyes;
        private final long timeoutMs;
        private long startTime = 0;

        public WaitForTagAction(Eyes eyes, long timeoutMs) {
            this.eyes = eyes;
            this.timeoutMs = timeoutMs;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            if (startTime == 0) {
                startTime = System.currentTimeMillis();
            }

            eyes.update();

            packet.put("AprilTag ID", eyes.detectedTag);

            boolean timedOut =
                    System.currentTimeMillis() - startTime >= timeoutMs;

            // keep running while no tag and not timed out
            return eyes.detectedTag == 22 && !timedOut;
        }
    }
    public class shooter {
        private final DcMotorEx motor1;



        public shooter(HardwareMap hardwareMap) {
            motor1 = hardwareMap.get(DcMotorEx.class, "rightFly");

            PIDFCoefficients pidfCoefficients = new PIDFCoefficients(75,0,0,13.2);
            motor1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);


        }


        public class SpinUp implements Action {
            private boolean initialized = false;


            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    motor1.setVelocity(-1200);
                    initialized = true;
                }

                return false;
            }


        }
        public Action spinUp() {
            return new SpinUp();
        }
        public class SpinDown implements Action {


            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                motor1.setVelocity(0);
                return false;
            }
        }
        public Action spinDown(){
            return new SpinDown();
        }
    }
    public class kick {
        private Servo kicker;


        public kick(HardwareMap hardwareMap) {
            kicker = hardwareMap.get(Servo.class, "ethan");
        }


        public class kickDown implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                kicker.setPosition(1);
                sleep(100);
                return false;
            }
        }
        public Action kickDown() {
            return new kickDown();
        }


        public class kickUp implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                kicker.setPosition(0);
                sleep(100);
                return false;
            }
        }
        public Action kickUp() {
            return new kickUp();
        }
    }
    public class combine {
        private final DcMotorEx intake;

        public combine(HardwareMap hardwareMap) {
            intake = hardwareMap.get(DcMotorEx.class, "bottomIntake");
            intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        }
        public class Intake implements Action {
            private boolean initialized = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    intake.setPower(0.5);
                    initialized = true;
                    sleep(500);
            }
                return false;
            }


        }
        public Action intake() {
            return new Intake();
        }


        public class Outtake implements Action {
            private boolean initialized = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    intake.setPower(-0.5);
                    initialized = true;
                }
                return false;
            }


        }
        public Action outtake() {
            return new Outtake();
        }


        public class Holdtake implements Action {


            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intake.setPower(0);
                return false;
            }
        }
        public Action holdtake(){
            return new Holdtake();
        }
    }
    public class spindex {
        private final DcMotorEx spindexer;
        private final DistanceSensor frontDistance;

        public spindex(HardwareMap hardwareMap) {
            spindexer = hardwareMap.get(DcMotorEx.class, "topIntake");
            spindexer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontDistance = hardwareMap.get(DistanceSensor.class, "frontSlot");
        }

        public class NextSlot implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    spindexer.setTargetPosition(spindexer.getCurrentPosition() + slotTicks);
                    spindexer.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    spindexer.setPower(.3);
                    initialized = true;
                    sleep(400);
                }
                return false;
            }
        }

        public Action nextSlot() {
            return new NextSlot();
        }

        public class PrevSlot implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    spindexer.setTargetPosition(spindexer.getCurrentPosition() - slotTicks);
                    spindexer.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    spindexer.setPower(.3);
                    initialized = true;
                }
                return false;
            }
        }

        public Action prevSlot() {
            return new PrevSlot();
        }

        public class AutoIntake implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {

                    initialized = true;
                }
                if (frontDistance.getDistance(DistanceUnit.CM) > 8) {
                    return true;
                } else {
                    return false;
                }
            }
        }
        public Action autoIntake() {
            return new AutoIntake();
        }
    }

    //Set up Classes for Trajectory + Actions
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(-56, 46, Math.toRadians(130));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        shooter shooter = new shooter(hardwareMap);
        kick kick = new kick(hardwareMap);
        combine combine = new combine(hardwareMap);
        spindex spindex = new spindex(hardwareMap);
        Eyes eyes = new Eyes(hardwareMap);

        while (!isStopRequested() && !opModeIsActive()) {
            int position = visionOutputPosition;
            telemetry.addData("Position during Init", position);
            telemetry.update();
            if (isStopRequested()) return;
        }
        int startPosition = visionOutputPosition;
        telemetry.addData("Starting Position", startPosition);
        telemetry.update();
        waitForStart();
// Trajectories
        TrajectoryActionBuilder visionSet = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(-35,15),Math.toRadians(190));
        TrajectoryActionBuilder shootSet1 = drive.actionBuilder(new Pose2d(-35,15,Math.toRadians(190)))
                .strafeToLinearHeading(new Vector2d(-30,20),Math.toRadians(130));
        TrajectoryActionBuilder intakeTopSet = drive.actionBuilder(new Pose2d(-30,20, Math.toRadians(130)))
                .strafeToLinearHeading(new Vector2d(-15,20),Math.toRadians(90));
        TrajectoryActionBuilder intakingTop = drive.actionBuilder(new Pose2d(-15,20, Math.toRadians(90)))
                .strafeTo(new Vector2d(-15,30))
                .strafeTo(new Vector2d(-15,52), new TranslationalVelConstraint(4));
        TrajectoryActionBuilder outsideSet = drive.actionBuilder(new Pose2d(-30,20,Math.toRadians(130)))
                .strafeTo(new Vector2d(0,40));
        TrajectoryActionBuilder shootSet2 = drive.actionBuilder(new Pose2d(-15,52,Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(-30,20), Math.toRadians(130));

        Action waitForTag = new WaitForTagAction(eyes,1500);
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
                new SleepAction(5)
                //waitForTag
        ));
        Action ballOrganize;
        if (eyes.detectedTag==21){
            ballOrganize = spindex.nextSlot();
        } else if (eyes.detectedTag==23) {
            ballOrganize = spindex.prevSlot();
        } else  {
            ballOrganize = new SleepAction(1);
        }
        Actions.runBlocking(new SequentialAction(
                new ParallelAction(
                        shootSet1.build(),
                        ballOrganize
                ),
                unloadBalls,
                intakeTopSet.build(),
                combine.intake(),
                new ParallelAction(
                        intakingTop.build(),
                        new SequentialAction(
                                spindex.autoIntake(),
                                spindex.nextSlot(),
                                spindex.autoIntake(),
                                spindex.nextSlot(),
                                spindex.autoIntake()
                        )
                ),
                combine.holdtake(),
                shooter.spinUp(),
                shootSet2.build(),
                unloadBalls,
                outsideSet.build()
        ));
    }
}
