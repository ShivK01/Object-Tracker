package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

//Class which controls the drivetrain
public class DriveSubsystem {
    //Declare all objects/variables
    private GyroSubsystem gyro;
    private PIDSubsystem PID;
    private OdometrySubsystem odo;
    private ElapsedTime timer;
    private DcMotor FR, FL, BR, BL;
    private double FRpower, FLpower, BRpower, BLpower, largest;
    private final double MULT = 0.8; //multiplier for power
    private final double DEGTRAD = Math.PI / 180; //constant to convert deg to rad
    private double xPow, yPow, power, robotDirection, newAngle;

    //Constructor for the class
    public DriveSubsystem(HardwareMap hardwareMap) {
        gyro = new GyroSubsystem(hardwareMap);
        PID = new PIDSubsystem();
        odo = new OdometrySubsystem(hardwareMap);
        timer = new ElapsedTime();
        //Set PID constants for the movements
        PID.KDerivative = 0;
        PID.KProportional = 0.2;
        PID.KIntegral = 0;

        //Initialize and set all motors
        FR = hardwareMap.get(DcMotor.class, "FR");
        FL = hardwareMap.get(DcMotor.class, "FL");
        BR = hardwareMap.get(DcMotor.class, "BR");
        BL = hardwareMap.get(DcMotor.class, "BL");

        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        FL.setDirection(DcMotorSimple.Direction.FORWARD);
        BR.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.FORWARD);

        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    //Method which outputs the power to the wheels
    //x is the power for latitude, y is the power for the longitude, z is the rotational power
    public void move(double x, double y, double z) {
        //Sum powers for overall power output to each wheel
        FRpower = x - y - z;
        FLpower = -x - y + z;
        BRpower = -x - y - z;
        BLpower = x - y + z;

        //scale all powers to range of -1< <1
        largest = Math.max(Math.max(Math.abs(FRpower), Math.abs(FLpower)), Math.max(Math.abs(BRpower), Math.abs(BLpower)));
        FRpower /= largest;
        FLpower /= largest;
        BRpower /= largest;
        BLpower /= largest;

        //Output the power (multiplied by the MULT which further scales power down)
        FR.setPower(FRpower * MULT);
        FL.setPower(FLpower * MULT);
        BR.setPower(BRpower * MULT);
        BL.setPower(BLpower * MULT);
    }

    //Method to drive at any angle but in a straight line. Field orientated
    //Requires an angle and distance in cm to be passed.
    //0 deg is right, 90 deg is forward, 180 deg is left, 270 deg is backward.
    //          90
    //      180     0
    //          270
    //Remember, these drive angles are fixed to the front of the robot.
    public void drive(double angle, double distance) {
        //Reset the odometry encoder position
        odo.reset();
        //While not in a ±1 cm buffer of the target distance
        while(odo.robotPos() < distance - 1 || odo.robotPos() > distance + 1){
            //Retrieve power using PID
            power = PID.PIDControl(distance, odo.robotPos()) / 5;
            //This line is for field orientation, or rather to keep the 0-90-180-270
            //  direction of the robot constant regardless or any turn the robot makes
            //Get the new angle the robot should go to (deg)
            newAngle = angle - gyro.getAngleDeg();
            //Break power into x and y vectors
            xPow = power * Math.cos(newAngle * DEGTRAD); //Take cos of the angle and times by power for x power
            yPow = power * Math.sin(newAngle * DEGTRAD); //Take sin of the angle and times by power for y power
            //Output powers to move method. z = 0 because we want to drive straight with no rotation
            move(xPow, yPow, 0);
        }
        //stop robot after
        stop();
    }

    //Method to drive straight at any angle for a given time
    //Time should be passed in milliseconds
    public void driveWithTime(double angle, double time) {
        power = 0.05;
        //Reset the timer
        timer.reset();
        newAngle = angle - gyro.getAngleDeg();
        //Break power into x and y vectors for driving at specified angle
        xPow = Math.cos(newAngle * DEGTRAD) * power;
        yPow = Math.sin(newAngle * DEGTRAD) * power;
        //While timer is less than time we want to drive for
        while(timer.milliseconds() < time){
            //Output power to move method
            move(xPow, yPow, 0);
        }
        //Stop robot after
        stop();
    }

    //Method turns the robot to a specified angle in degrees
    //Pass angle in deg
    public void turn(double z) {
        //Pull current robot angle from gyro into robotDirection
        robotDirection = gyro.getAngleDeg();
        //While robot is not within ±1 deg buffer of specified angle
        while(robotDirection > z + 1 || robotDirection < z - 1){
            //Refresh the robot direction
            robotDirection = gyro.getAngleDeg();
            //This while loop allows us to not have to write a bunch of conditions for the different
            //cases of robot angle vs angle goal could be
            //Needed since gyro returns angle from -180 to 180 and not 0 to 360
            //This loop allows us to pass angles from 0 to 360 and have robot turn to it still referencing the gyro
            while(z - robotDirection > 180 || z - robotDirection < -180){
                if(z - robotDirection > 180){
                    robotDirection += 360;
                }else{
                    robotDirection -= 360;
                }
            }
            //Get power from PID method
            power = PID.PIDControl(z, robotDirection);
            //Output power as rotational power to move method
            move(0, 0, power);
        }
        stop();
    }

    //Method to stop motors
    //Made for ease
    public void stop(){
        move(0, 0, 0);
    }
}
