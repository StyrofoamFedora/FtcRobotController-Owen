package org.firstinspires.ftc.teamcode.autonomous;

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
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
@Autonomous(name = "SixBall", group = "Autonomous")
public class SixBallAutoTest extends LinearOpMode {
    int visionOutputPosition = 0;

    public class shooter {
        private DcMotorEx motor1;
        private DcMotorEx motor2;

        public shooter(HardwareMap hardwareMap) {
            motor1 = hardwareMap.get(DcMotorEx.class, "leftFly");
            motor2 = hardwareMap.get(DcMotorEx.class, "rightFly");

        }

        public class SpinUp implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    motor1.setPower(-0.5);
                    motor2.setPower(0.5);
                    initialized = true;
                }
                double vel1 = Math.abs(motor1.getVelocity());
                packet.put("shooterVelocity", vel1);
                return vel1 < 10_000.0;
            }

        }
        public Action spinUp() {
            return new SpinUp();
        }
        public class SpinDown implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    motor1.setPower(0);
                    motor2.setPower(0);
                    initialized = true;
                }
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
                kicker.setPosition(-.5);
                return false;
            }
        }
        public Action kickDown() {
            return new kickDown();
        }

        public class kickUp implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                kicker.setPosition(1);
                return false;
            }
        }
        public Action kickUp() {
            return new kickUp();
        }
    }
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(7.00, -70.00, Math.toRadians(90.00));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        shooter shooter = new shooter(hardwareMap);
        kick kick = new kick(hardwareMap);
       // AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder();
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
//Pick apriltagid
/*
        Action tripletChosen;
        if (apriltagid == 23){
            tripletChosen = intakeTopSet.build();
        } else if (apriltagid == 22) {
            tripletChosen = intakeMiddleSet.build();
        } else {
            tripletChosen = intakeBottomSet.build();
        }
*/
//Stuff That's run
        Actions.runBlocking(new SequentialAction(
                shootSet1.build(),
                new ParallelAction(
                        shooter.spinUp(),
                        new SequentialAction(
                                new SleepAction(1),
                                kick.kickUp(),
                                new SleepAction(1),
                                kick.kickDown(),
                                new SleepAction(1),
                                kick.kickUp(),
                                new SleepAction(1),
                                kick.kickDown(),
                                new SleepAction(1),
                                kick.kickUp(),
                                new SleepAction(1),
                                kick.kickDown(),
                                shooter.spinDown()
                        )
                ),
                //tripletChosen,
                new ParallelAction(
                        //Start Intake
                        new SequentialAction(
                                intaking.build()
                                //Stop Intake
                                )
                )
        ));
    }
}
