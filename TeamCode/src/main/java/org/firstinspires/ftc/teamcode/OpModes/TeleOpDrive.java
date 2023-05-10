package org.firstinspires.ftc.teamcode.OpModes;

import static java.lang.Thread.sleep;

import android.os.Build;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.CameraPipeline;
import org.firstinspires.ftc.teamcode.subsystems.CameraSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

import java.util.concurrent.CompletableFuture;
import java.util.concurrent.Executor;
import java.util.concurrent.Executors;

//This teleOp mode is to control the entire robot with the controller
@TeleOp(name = "TeleOp")
public class TeleOpDrive extends OpMode {
    //Declare used classes/objects
    private DriveSubsystem drive;
    private TurretSubsystem turret;
    private CameraSubsystem camera;
    private CameraPipeline pipeline;
    //Declare async for multithreading
    Executor executor = Executors.newFixedThreadPool(5);
    //Declare used variables
    private final double PROP = 0.3; //To scale joystick input down
    private boolean follow, state, lastState; //state vars to control button presses. follow to control camera mode

    //Runs during init only
    @Override
    public void init(){
        drive = new DriveSubsystem(hardwareMap);
        turret = new TurretSubsystem(hardwareMap);
        camera = new CameraSubsystem(hardwareMap);
        pipeline = camera.getPipeline();
        telemetry.setMsTransmissionInterval(50);
        follow = state = lastState = false;
    }

    //Runs continuously during play
    @Override
    public void loop(){
        /*This is to control the robot to drive anywhere based on controller input
          The right stick:
            * up will move robot forward
            * down will move robot backward
            * right will strafe robot right
            * left will strafe robot left
          The left stick:
            * right will rotate robot clockwise
            * left will rotate robot counterclockwise
            * up and down don't do anything
         */
        drive.move(gamepad1.right_stick_x * PROP, -gamepad1.right_stick_y * PROP, -gamepad1.left_stick_x * PROP);

        //This code is so that when the a button is pressed, the robot doesn't continuously read presses
        //You must release button after press and then press it again to register a new press. If you hold the button down,
        // its the same as pressing it once.
        state = gamepad1.a;
        if(state != lastState) {
            if(state){
                follow = !follow;
            }
        }
        lastState = state;

        //If follow is true, the camera will move itself to center on a yellow object in its view
        if(follow && Build.VERSION.SDK_INT >= Build.VERSION_CODES.N){
            //Run the followYellow method as multithread so you can still drive robot while camera follows yellow object
            CompletableFuture.runAsync(() -> followYellow(), executor);
            telemetry.addLine("Camera: follow");
        //If follow is false, have the camera move because of user input
        }else{
            //Pressing x moves camera counterclockwise
            if(gamepad1.x){
                turret.rotate(-0.15);
            //Pressing b moves camera clockwise
            }else if(gamepad1.b){
                turret.rotate(0.15);
            }else{
                //Pressing anything else or nothing doesn't move the camera
                turret.stop();
            }
            telemetry.addLine("Camera: user");
        }
        telemetry.update();
    }

    //Method to have camera hold focus on yellow object in sight
    //Keep in mind the condition in the pipeline for camera to register an object (max >= 7)
    private void followYellow(){
        //If object is in left of camera, rotate turret counterclockwise
        if(pipeline.loc == 0){
            turret.rotate(-0.2);
        //If object is in right of camera, rotate turret clockwise
        }else if(pipeline.loc == 2){
            turret.rotate(0.2);
        }else{
            turret.stop();
        }
    }
}
