package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

//Class hold the code for the camera to function
//Camera is dependent on the pipeline to function correctly
public class CameraSubsystem {
    //Declare objects and variables
    public OpenCvCamera camera;
    private CameraPipeline pipeline;
    int cameraMonitorViewId;

    //Constructor for the class
    public CameraSubsystem(HardwareMap hardwareMap){
        //Initialize the camera
        pipeline = new CameraPipeline();
        cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId","id",hardwareMap.appContext.getPackageName());
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        //set the camera pipeline
        camera.setPipeline(pipeline);
        //code for streaming the camera view to phone when opened in Init.
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener(){
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });
    }

    //Method returns the pipeline so that it can be used in other classes
    //you want one pipeline to be used across all classes so you can reference what the camera sees in other classes
    public CameraPipeline getPipeline() {
        return pipeline;
    }
}

