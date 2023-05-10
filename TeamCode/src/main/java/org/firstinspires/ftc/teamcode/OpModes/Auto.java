package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.CameraPipeline;
import org.firstinspires.ftc.teamcode.subsystems.CameraSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

import java.util.concurrent.Executor;
import java.util.concurrent.Executors;

//This is the auto class for the robot
//The code written will drive the robot 1.3m forward, then turn right 90deg and then center the
// robot relative to the camera lens on the yellow pole
@Autonomous(name="Auto", group="Robot")
public class Auto extends LinearOpMode{
    //Declare used classes/objects
    private TurretSubsystem turret;
    private CameraSubsystem camera;
    private CameraPipeline pipeline;
    private DriveSubsystem driveCommand;

    @Override
    public void runOpMode() {
        //Initialize everything
        turret = new TurretSubsystem(hardwareMap);
        camera = new CameraSubsystem(hardwareMap);
        driveCommand = new DriveSubsystem(hardwareMap);
        pipeline = camera.getPipeline();
        Executor executor = Executors.newFixedThreadPool(5); //For multithreading
        waitForStart(); //Code will run up to here and wait during init. Waits here until start is pressed

        //Drive forwards 130cm and then turn right
        driveCommand.drive(90, 130);
        driveCommand.turn(285); //instead of turning right to 270, go to 285 because there seems to be some natural error
        while(true) {
            //Print location of the pole (0, 1, 2) corresponding to pipeline
            telemetry.addData("Location", pipeline.loc);
            telemetry.update();
            //If pole is in left of camera, move robot left (drive at 90deg for 50msec). Remember field orientation
            if (pipeline.loc == 0) {
                //turret.rotate(-0.2); You could use this to rotate the turret instead
                driveCommand.driveWithTime(90, 50);
            //If pole is in right of camera, move robot right (drive at 270deg for 50msec). Remember field orientation
            }else if (pipeline.loc == 2) {
                //turret.rotate(0.2); You could use this to rotate the turret instead
                driveCommand.driveWithTime(270, 50);
            //Otherwise stop. Should stop if robot is centered on pole or no pole detected
            }else{
                //turret.stop();
                driveCommand.stop();
            }
        }
        /**
         * Keep in mind you can change this code to suit what you want
         * You can have robot drive different distances or and turn somewhere else
         * You can have the turret rotate instead of the robot move to center on pole
         * You can add more code to center on a pole and then drive to another pole and center on it
         * This code is just a base to demonstrate.
         * Feel free to clone this repo and change how and where you use the code to center the robot
         */
    }
}

