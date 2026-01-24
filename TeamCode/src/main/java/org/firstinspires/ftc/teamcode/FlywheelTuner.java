package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp
public class FlywheelTuner extends OpMode {

    public DcMotorEx spindexer;


    int TargetVelocity = 0;
    double F = 0;
    double P = 0;
    double[] stepSizes = {10.0,1.0,0.1,0.01,0.001};
    int stepIndex = 1;
    double CPR = 537.7;
    double slotTicks = CPR/3;
    int currentIntTick;
    double currentTick = 0;


    @Override
    public void init(){

        DcMotorEx spindexer = hardwareMap.get(DcMotorEx.class,"topIntake");
        PIDFCoefficients DrivePIDFCoefficients = new PIDFCoefficients(P,0,0,F);
        spindexer.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER,DrivePIDFCoefficients);

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
            currentTick-=slotTicks;
            currentIntTick = (int)Math.round(currentTick);
            spindexer.setTargetPosition(currentIntTick);
            spindexer.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            spindexer.setPower(.3);
        }
        if(gamepad1.rightBumperWasPressed()){
            currentTick+=slotTicks;
            currentIntTick = (int)Math.round(currentTick);
            spindexer.setTargetPosition(currentIntTick);
            spindexer.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            spindexer.setPower(.3);
        }
        PIDFCoefficients DrivePIDFCoefficients = new PIDFCoefficients(P,0,0,F);
        spindexer.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER,DrivePIDFCoefficients);
        double FRCurrentVelocity = spindexer.getVelocity();
        double FRerror = TargetVelocity - FRCurrentVelocity;

        spindexer.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        spindexer.setPower(.3);

        telemetry.addData("Target Velocity",TargetVelocity);
        telemetry.addData("FRCurrent Velocity",FRCurrentVelocity);
        telemetry.addData("FRError",FRerror);
        telemetry.addData("Tuning P(Up/Down)",P);
        telemetry.addData("Tuning F(Left/Right)",F);
        telemetry.addData("Step Size(B)",stepSizes[stepIndex]);
        }
}
