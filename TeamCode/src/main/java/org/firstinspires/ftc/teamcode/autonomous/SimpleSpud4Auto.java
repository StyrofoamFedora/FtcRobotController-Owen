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
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

//Class Define
@Config
@Autonomous(name = "SixBallVision", group = "Autonomous")
public class SimpleSpud4Auto extends LinearOpMode {
    //Create Default Variables for Vision Sets
    int visionOutputPosition = 0;
    int apriltagid = 23;
    int CPR = 538;
    int slotTicks = CPR/3;
    //Set up Actions
    public class eyes{
        private WebcamName cam;
        private AprilTagProcessor aprilTagProcessor;
        public eyes(HardwareMap hardwareMap){
            cam = hardwareMap.get(WebcamName.class, "cam");
        }
        public class Detect implements Action{
            public AprilTagProcessor aprilTagProcessor;
            public Detect(AprilTagProcessor processor) {
                this.aprilTagProcessor = processor;
            }
            public boolean run(@NonNull TelemetryPacket packet) {
                List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
                if (!detections.isEmpty()) {
                    AprilTagDetection ob = detections.get(0);
                    apriltagid = ob.id;
                    packet.put("ATID", apriltagid);
                }
                return false;
            }
        }
        public Action detect() {return new Detect(aprilTagProcessor);}
    }
    public class shooter {
        private final DcMotorEx motor1;



        public shooter(HardwareMap hardwareMap) {
            motor1 = hardwareMap.get(DcMotorEx.class, "rightFly");



        }


        public class SpinUp implements Action {
            private boolean initialized = false;


            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    motor1.setPower(0.55);
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
                motor1.setPower(0);
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

        public spindex(HardwareMap hardwareMap) {
            spindexer = hardwareMap.get(DcMotorEx.class, "topIntake");
            spindexer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        }
        public class NextSlot implements Action {
            private boolean initialized = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    spindexer.setTargetPosition(spindexer.getCurrentPosition()+slotTicks);
                    spindexer.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    spindexer.setPower(.4);
                    initialized = true;
                    sleep(500);
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
                    spindexer.setTargetPosition(spindexer.getCurrentPosition()-slotTicks);
                    spindexer.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    spindexer.setPower(.4);
                    initialized = true;
                }
                return false;
            }
        }
        public Action prevSlot() {
            return new PrevSlot();
        }
    }

    //Set up Classes for Trajectory + Actions
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(7.00, -70.00, Math.toRadians(90.00));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        shooter shooter = new shooter(hardwareMap);
        kick kick = new kick(hardwareMap);
        combine combine = new combine(hardwareMap);
        spindex spindex = new spindex(hardwareMap);
        eyes eyes = new eyes(hardwareMap);
        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder().build();
        VisionPortal visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "cam"))
                .addProcessor(tagProcessor)
                .build();
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
                .strafeToLinearHeading(new Vector2d(7,-50), Math.toRadians(85));
        TrajectoryActionBuilder shootSet1 = drive.actionBuilder(new Pose2d(7,-50, Math.toRadians(85)))
                .strafeTo(new Vector2d(7,-60))
                .strafeToLinearHeading(new Vector2d(-7,-35), Math.toRadians(145))
                .waitSeconds(0.5);
        TrajectoryActionBuilder intakeTopSet = drive.actionBuilder(new Pose2d(-7,-35, Math.toRadians(145)))
                .strafeToLinearHeading(new Vector2d(5,-30),Math.toRadians(180));
        TrajectoryActionBuilder intakeMiddleSet = drive.actionBuilder(new Pose2d(-7,-35, Math.toRadians(145)))
                .strafeTo(new Vector2d(5,-15))
                .strafeToLinearHeading(new Vector2d(5,10),Math.toRadians(180));
        TrajectoryActionBuilder intakeBottomSet = drive.actionBuilder(new Pose2d(-7,-35, Math.toRadians(145)))
                .strafeTo(new Vector2d(5,25))
                .strafeToLinearHeading(new Vector2d(5,50),Math.toRadians(180));
        TrajectoryActionBuilder intaking = drive.actionBuilder(new Pose2d(5,0, Math.toRadians(180)))
                .lineToX(-50);
        TrajectoryActionBuilder shootSet2 = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(7,-60), Math.toRadians(145));
//Pick apriltagid and set as Triplet
        Action tripletChosen;
        if (apriltagid == 23){
            tripletChosen = intakeTopSet.build();
        } else if (apriltagid == 22) {
            tripletChosen = intakeMiddleSet.build();
        } else {
            tripletChosen = intakeBottomSet.build();
        }
//Stuff That's run
        Actions.runBlocking(new SequentialAction(
 //               visionSet.build(),
                shootSet1.build(),
                new ParallelAction(
                        shooter.spinUp(),
                    new SequentialAction(
                        new SleepAction(1),
                        kick.kickUp(),
                        new SleepAction(0.1),
                        kick.kickDown(),
                        new SleepAction(1),
                        spindex.nextSlot(),
                        new SleepAction(1),
                        kick.kickUp(),
                        new SleepAction(0.1),
                        kick.kickDown(),
                        new SleepAction(1),
                        spindex.nextSlot(),
                        new SleepAction(1),
                        kick.kickUp(),
                        new SleepAction(0.1),
                        kick.kickDown(),
                        new SleepAction(1),
                        spindex.nextSlot()
                    )
                ),
                new SleepAction(1),
                shooter.spinDown(),
                tripletChosen
        ));
    }
}
