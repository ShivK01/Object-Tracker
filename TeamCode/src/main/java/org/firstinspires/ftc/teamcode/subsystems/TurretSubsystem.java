package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.checkerframework.checker.units.qual.A;

//Class that controls the turret subsystem and its movements
public class TurretSubsystem {
    //Declare all used variables/objects
    private DcMotor turret;
    private double power;
    private PIDSubsystem PID;
    //This constant is used to convert degrees to ticks
    private final double ANGLETOTICK = (double)780 / 360;

    public TurretSubsystem(HardwareMap hardwareMap){
        //Motor initialization and setup
        turret = hardwareMap.get(DcMotor.class, "turret");
        turret.setDirection(DcMotorSimple.Direction.FORWARD);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Setting PID object constants
        PID = new PIDSubsystem();
        PID.KProportional = 0.01;
        PID.KDerivative = 0;
        PID.KIntegral = 0;
    }

    //Method rotates the turret provided a motor power
    //Positive power turret goes clockwise. Negative power turret goes counterclockwise
    public void rotate(double power){
        turret.setPower(-power);
    }

    //Method which stops the turret
    public void stop(){
        turret.setPower(0);
    }

    /*
    -Method which rotates the turret to a given angle.
    -The 0 point is where ever the turret is pointed before the OpMode is started.
    -Use the teleop mode to adjust turret to a set reference position before autonomous
    -This method converts the angle to ticks and then references turret angle through ticks
      because there is no gyroscope on the turret.
    */
    public void rotateTo(double angle){
        //While not in a 1 tick ±buffer of the desired angle, correct the turret angle
        while(turret.getCurrentPosition() < convert(angle) - 1 || turret.getCurrentPosition() > convert(angle) + 1){
            //Use PID in terms of ticks to retrieve power for the turret
            power = PID.PIDControl(convert(angle), turret.getCurrentPosition());
            turret.setPower(power/2);
        }
        stop();//Stop the turret after angle has been reached
    }

    //Method converts an angle to ticks for the turret and outputs this
    //Simply makes the code a bit cleaner in the rotateTo method
    private double convert(double angle){
        return angle * ANGLETOTICK;
    }
}
