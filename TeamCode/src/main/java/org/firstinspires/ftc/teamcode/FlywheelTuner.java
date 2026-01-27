package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp
public class FlywheelTuner extends OpMode {

    //How to use:
    //First, Tune the Feed Forward.
    //Use the B button to change the step size (First to 10, then 1, and so on), and increase F using the right D-Pad.
    //Once the Error turns possitive, you've gone to far (This is good). Now decrease F using the left D-pad using a...
    //          smaller step size
    //Repeat this process until your error balanced out at as close to 0 as possible.
    //Next, tune the Proportional.
    //Increase P using Up arrow.
    //This one is harder to get percise, but increase it until just before it starts to occilate the error.
    //This is what makes the flywheel speed back up, so you can also shoot a ball through it to test if it's enough of a value.
    //If both Values are satisfactory, you should be good to go with just those two.
    //If you find it neccesary to include other values (I or D), you can replace F and P in the code (Remember to set F and P....
    //          to the values you already found), This shouldn't be neccesary, especially for I, but could be useful if it's....
    //          still just a bit off.

    //To use in code, just set:
    //      PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P,0,0,F);
    //        fly.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
    // right before the opmode, and use setVelocity instead of setPower
    // You may have to have it in your robot class, idrk how that works

    public DcMotorEx fly;
    //Target Velocity should be in the thousands (It's in Ticks per Second, found on your motor)
    //Using a base REV motor (28 Ticks per Revolution), at likely a few thousand RPM (3000) gets us a...
    //          TargetVelocity of 1400
    int TargetVelocity = 1400;
    //Feed Forward in this case will likely be in the teens or twenties (ours is 13), but I expect yours to...
    //          be more, likely around 20 (you have more mass to move, slightly more friction, and a higher RPM)
    double F = 0;
    //Proportional will be around 100 (ours is 75), This will also likely be more, for the same reasons as above.
    double P = 0;
    double[] stepSizes = {10.0,1.0,0.1,0.01,0.001};
    int stepIndex = 1;



    @Override
    public void init(){

        DcMotorEx fly = hardwareMap.get(DcMotorEx.class,"fly");
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P,0,0,F);
        fly.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER,pidfCoefficients);

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

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P,0,0,F);
        fly.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER,pidfCoefficients);
        double CurrentVelocity = fly.getVelocity();
        double error = TargetVelocity - CurrentVelocity;


        telemetry.addData("Target Velocity",TargetVelocity);
        telemetry.addData("Current Velocity",CurrentVelocity);
        telemetry.addData("Error",error);
        telemetry.addData("Tuning P(Up/Down)",P);
        telemetry.addData("Tuning F(Left/Right)",F);
        telemetry.addData("Step Size(B)",stepSizes[stepIndex]);
        }
}
