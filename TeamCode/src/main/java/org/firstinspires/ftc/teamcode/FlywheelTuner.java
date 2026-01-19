package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import java.lang.annotation.Target;

@TeleOp
public class FlywheelTuner extends OpMode {
    public DcMotorEx rightFly;
    double TargetVelocity = 1200;
    double F = 0;
    double P = 0;
    double[] stepSizes = {10.0,1.0,0.1,0.01,0.001};
    int stepIndex = 1;


    @Override
    public void init(){
        rightFly = hardwareMap.get(DcMotorEx.class, "rightFly");
        rightFly.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFly.setDirection(DcMotorSimple.Direction.REVERSE);

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P,0,0,F);
        rightFly.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        telemetry.addLine("Init complete");
    }
    @Override
    public void loop(){
        if (gamepad1.bWasPressed()){
            stepIndex = (stepIndex +1) % stepSizes.length;
        }
        if (gamepad1.dpadLeftWasPressed()){
            F -= stepSizes[stepIndex];
        }
        if (gamepad1.dpadRightWasPressed()) {
            F += stepSizes[stepIndex];
        }
        if (gamepad1.dpadDownWasPressed()){
            P -= stepSizes[stepIndex];
        }
        if (gamepad1.dpadUpWasPressed()) {
            P += stepSizes[stepIndex];
        }
        if (gamepad1.leftBumperWasPressed()) {
            TargetVelocity -= 10;
        }
        if(gamepad1.rightBumperWasPressed()){
            TargetVelocity += 10;
        }
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P,0,0,F);
        rightFly.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER,pidfCoefficients);
        rightFly.setVelocity(TargetVelocity);
        double CurrentVelocity = rightFly.getVelocity();
        double error = TargetVelocity - CurrentVelocity;

        telemetry.addData("Target Velocity",TargetVelocity);
        telemetry.addData("Current Velocity",CurrentVelocity);
        telemetry.addData("Error",error);
        telemetry.addData("Tuning P(Up/Down)",P);
        telemetry.addData("Tuning F(Left/Right)",F);
        telemetry.addData("Step Size(B)",stepSizes[stepIndex]);
        }
}
