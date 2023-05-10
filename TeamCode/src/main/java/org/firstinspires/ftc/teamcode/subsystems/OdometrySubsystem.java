package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

//Class which controls the odometry for the robot
public class OdometrySubsystem {
    //Declare odometry wheels as motors
    public DcMotor horizontal;
    public DcMotor vertical;
    //Constant to convert from encoder ticks to cm.
    //2 * pi * radius of wheel / ticks per revolution
    private final double tickToCm = 2.0 * Math.PI * 1.75 / 8192;

    //Constructor for the class
    public OdometrySubsystem(HardwareMap hardwareMap){
        //Set all motor config and modes
        horizontal = hardwareMap.dcMotor.get("horizontal");
        vertical = hardwareMap.dcMotor.get("vertical");
        horizontal.setDirection(DcMotorSimple.Direction.FORWARD);
        vertical.setDirection(DcMotorSimple.Direction.FORWARD);
        horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        vertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontal.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        vertical.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    //Method to reset the encoders of the odometry wheels
    public void reset(){
        horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        vertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontal.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        vertical.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    //Method which returns the distance that the robot has travelled
    public double robotPos(){
        //The horizontal wheel tracks the distance travelled in x-axis
        //The vertical wheel tracks the distance travelled in y-axis
        //The hypotenuse of the triangle made with the legs of the horizontal and
        //  vertical distances is the distance the robot has travelled.
        //Make sure to convert from encoder ticks to cm
        return Math.sqrt(Math.pow(horizontal.getCurrentPosition() * tickToCm, 2)
                + Math.pow(vertical.getCurrentPosition() * tickToCm, 2));
    }
}
