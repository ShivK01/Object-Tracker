package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.CameraPipeline;
import org.firstinspires.ftc.teamcode.subsystems.CameraSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

//This mode is more of a test teleOp mode for the camera
//You can use it to have the camera follow a yellow object that you move.
//The robot itself can't move
@TeleOp(name = "CameraMove")
public class CameraMove extends OpMode {
    //Declare objects
    private TurretSubsystem turret;
    private CameraSubsystem camera;
    private CameraPipeline pipeline;

    @Override
    public void init(){
        turret = new TurretSubsystem(hardwareMap);
        camera = new CameraSubsystem(hardwareMap);
        pipeline = camera.getPipeline();
        telemetry.setMsTransmissionInterval(50);
    }

    @Override
    public void loop(){
        //If object is in left of camera, rotate turret counterclockwise
        if(pipeline.loc == 0){
            turret.rotate(-0.1);
        //If object is in right of camera, rotate turret clockwise
        }else if(pipeline.loc == 2){
            turret.rotate(0.1);
        }else{
            turret.stop();
        }
        telemetry.addData("Location: ", pipeline.loc);
        telemetry.update();
    }
}
