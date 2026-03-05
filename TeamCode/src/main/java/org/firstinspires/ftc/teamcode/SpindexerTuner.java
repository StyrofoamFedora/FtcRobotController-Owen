package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class SpindexerTuner extends OpMode {

    double P = 0;
    double I = 0;
    double D = 0;
    double increaser;
    int type;
    double[] stepSizes = {10.0,1.0,0.1,0.01,0.001};
    int stepIndex = 1;
    double currentTick;
    double power = 0.5;
    int slots = 0;

    public DcMotorEx spindex;
    public Servo RGB;
    @Override
    public void init(){
        RGB = hardwareMap.servo.get("rgb");
        spindex = hardwareMap.get(DcMotorEx.class,"topIntake");
        spindex.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        spindex.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        spindex.setTargetPositionTolerance(2);
        spindex.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spindex.setTargetPosition(0);
        PIDCoefficients pidCoefficients = new PIDCoefficients(P,I,D);
        spindex.setPIDCoefficients(DcMotorEx.RunMode.RUN_TO_POSITION,pidCoefficients);

        telemetry.addLine("Init complete");
    }
    @Override
    public void loop(){
        if (gamepad1.bWasPressed()){
            stepIndex = (stepIndex +1) % stepSizes.length;
        }
        if (gamepad1.dpadLeftWasPressed()){
            type -= 1;
        }
        if (gamepad1.dpadRightWasPressed()) {
            type += 1;
        }
        if (gamepad1.dpadDownWasPressed()){
            increaser = -stepSizes[stepIndex];
        }
        if (gamepad1.dpadUpWasPressed()) {
            increaser = stepSizes[stepIndex];
        }
        if(gamepad1.aWasPressed()){slots = 1;}
        type %= 3;

        if(type == 0){P+=increaser;}
        else if (type == 1) {I+=increaser;}
        else if (type == 2) {D+=increaser;}

        PIDCoefficients pidCoefficients = new PIDCoefficients(P,I,D);
        spindex.setPIDCoefficients(DcMotorEx.RunMode.RUN_TO_POSITION,pidCoefficients);
        double CurrentPos = spindex.getCurrentPosition();
        double CPR = 537.7;
        double slotTicks = CPR/3;
        currentTick -=(slotTicks*slots);
        double error = currentTick - CurrentPos;
        int currentIntTick = (int)Math.round(currentTick);
        spindex.setTargetPosition(currentIntTick);
        spindex.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        spindex.setPower(power);

        if(spindex.isBusy()){RGB.setPosition(0.28);} else{
            RGB.setPosition(.5);
        }
        telemetry.addData("Target Pos", currentTick);
        telemetry.addData("Current Pos",CurrentPos);
        telemetry.addData("Error",error);
        telemetry.addData("Tuning P",P);
        telemetry.addData("Tuning I",I);
        telemetry.addData("Tuning D",D);
        telemetry.addData("Step Size(B)",stepSizes[stepIndex]);

        increaser = 0;
        slots = 0;
        }
}
