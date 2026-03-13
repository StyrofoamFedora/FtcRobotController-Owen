package org.firstinspires.ftc.teamcode.autonomous;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import java.util.List;

@Config
public class TestingAutoRobot{
    public static class Eyes {
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
        public void update() {
            List<AprilTagDetection> detections = aprilTag.getDetections();
            if (!detections.isEmpty()) {
                lastDetection = detections.get(0);
                detectedTag = lastDetection.id;
            }
        }

    }
    public static class WaitForTagAction implements Action {
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
            boolean timedOut = System.currentTimeMillis() - startTime >= timeoutMs;
            // keep running while no tag and not timed out
            return eyes.detectedTag == -1 && !timedOut;
        }
    }
    public static class shooter {
        private final DcMotorEx fly;
        public shooter(HardwareMap hardwareMap) {
            fly = hardwareMap.get(DcMotorEx.class, "rightFly");
            PIDFCoefficients pidfCoefficients = new PIDFCoefficients(80, 0, 0, 13.2);
            fly.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        }
        public class SpinUpClose implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                PIDFCoefficients pidfCoefficients = new PIDFCoefficients(80, 0, 0, 13.2);
                fly.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
                fly.setVelocity(1150);
                return false;
            }
        }public Action spinUpClose() {
            return new SpinUpClose();
        }
        public class SpinUpFar implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                PIDFCoefficients pidfCoefficients = new PIDFCoefficients(85, 0, 0, 15);
                fly.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
                fly.setVelocity(1650);
                return false;
            }
        }public Action spinUpFar() {
            return new SpinUpFar();
        }
        public class Stop implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                fly.setVelocity(0);
                return false;
            }
        }public Action stop() {
            return new Stop();
        }
    }
    public static class combine {
        private final DcMotorEx intake;
        public combine(HardwareMap hardwareMap) {
            intake = hardwareMap.get(DcMotorEx.class, "bottomIntake");
            intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        public class Intake implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intake.setPower(0.75);
                return false;
            }
        }public Action intake() {
            return new Intake();
        }
        public class Outtake implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intake.setPower(-0.5);
                return false;
            }
        }public Action outtake() {
            return new Outtake();
        }
        public class Holdtake implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intake.setPower(0);
                return false;
            }
        }public Action holdtake(){return new Holdtake();}
    }
    public static class intakeLock {
        private final Servo lock;
        public intakeLock(HardwareMap hardwareMap) {
            lock = hardwareMap.get(Servo.class, "lock");
        }
        public class Lock implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                lock.setPosition(0.85);
                return false;
            }
        }public Action lock() {return new Lock();}
        public class Unlock implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                lock.setPosition(0.4);
                return false;
            }
        }public Action unlock() {return new Unlock();}
    }
    public static class spindex{
        DcMotorEx spindexer;
        DistanceSensor frontDistance;
        double slotTicks = 537.7/3;
        double currentTick;
        public spindex(HardwareMap hardwareMap) {
            spindexer = hardwareMap.get(DcMotorEx.class, "topIntake");
            spindexer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontDistance = hardwareMap.get(DistanceSensor.class,"frontSlot");
            spindexer.setTargetPosition(spindexer.getCurrentPosition());
            currentTick = spindexer.getCurrentPosition();
        }

        public class Unload implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                currentTick +=(slotTicks*6);
                int currentIntTick = (int)Math.round(currentTick);
                spindexer.setTargetPosition(currentIntTick);
                spindexer.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                spindexer.setPower(.6);
                return false;
            }
        }public Action unload() {return new Unload();}
        public class PrevSlot implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                currentTick -= slotTicks;
                int currentIntTick = (int)Math.round(currentTick);
                spindexer.setTargetPosition(currentIntTick);
                spindexer.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                spindexer.setPower(.4);
                return false;
            }
        }public Action prevSlot() {return new PrevSlot();}
        public class WaitForBall implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                ElapsedTime autoIntakeTimer = new ElapsedTime();
                autoIntakeTimer.reset();
                return (frontDistance.getDistance(DistanceUnit.CM) > 8) || (autoIntakeTimer.seconds()>1);
            }
        }public Action waitForBall() {return new WaitForBall();}
    }
}
