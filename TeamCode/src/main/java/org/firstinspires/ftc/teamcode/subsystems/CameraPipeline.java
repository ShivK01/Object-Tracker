package org.firstinspires.ftc.teamcode.subsystems;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class CameraPipeline extends OpenCvPipeline {
    //White colour scalar for the detection boxes on the camera view
    private static final Scalar WHITE = new Scalar(255, 255, 255);
    //These scalars are for filtering colours.
    //General low and high bounds to filter out of (for yellow since pole)
    private final Scalar lowHSV = new Scalar(20, 70, 80);
    private final Scalar highHSV = new Scalar(32, 255, 255);
    //Strict low and high bounds for more accuracy (for yellow since pole)
    private final Scalar strictLowHSV = new Scalar(0, 130, 80);
    private final Scalar strictHighHSV = new Scalar(255, 255, 255);

    /** Keep in mind that if you wanted to detect and filter for a different colour, like red or blue,
        you would have to change the low and high HSV scalars to that particular range of colour in HSV.
        Use a colour picker in HSV to help you
    **/

    //Points for the left rectangle
    Point topLeftL = new Point(1, 1);
    Point bottomRightL = new Point(300, 447);
    //Point for the middle rectangle
    Point topLeftM = new Point(301, 1);
    Point bottomRightM = new Point(500, 447);
    //Points for the right rectangle
    Point topLeftR = new Point(501, 1);
    Point bottomRightR = new Point(799, 447);

    //Declare the mats(since mats are used as the rectangles)
    //left, middle and right will divide the camera into 3 areas to position the pole in camera view
    Mat left = new Mat();
    Mat middle = new Mat();
    Mat right = new Mat();
    Mat output = new Mat();

    //Declare variables for pole positioning
    private double leftAvg, middleAvg, rightAvg, max;
    public int loc;

    //Method which runs automatically assuming the camera object is declared and initialized in the OpMode
    //(remember the pipeline is set in the camera object)
    //The parameter input is passed automatically from the camera
    @Override
    public Mat processFrame(Mat input) {
        //Convert colour mode to HSV from RGB
        Mat HSV = new Mat();
        Imgproc.cvtColor(input, HSV, Imgproc.COLOR_RGB2HSV);

        //Filter the colour between out low and high boundaries. Turns black and white
        Mat thresh = new Mat();
        Core.inRange(HSV, lowHSV, highHSV, thresh);

        //Apply the thresh mask onto HSV mat to colour in black and white mat with HSV
        Mat masked = new Mat();
        Core.bitwise_and(HSV, HSV, masked, thresh);

        //Get HSV values of white thresh values
        Scalar avg = Core.mean(masked, thresh);
        //Scale the avg saturation to 150
        Mat scaledMask = new Mat();
        masked.convertTo(scaledMask, -1, 150/avg.val[1], 0);

        //Filter again between stricter boundaries. Narrows down to pole. Becomes black and white again
        Mat scaledThresh = new Mat();
        Core.inRange(scaledMask, strictLowHSV, strictHighHSV, scaledThresh);

        //Apply the scaledThresh mask onto out HSV mat again to recolour
        Mat yellowMask = new Mat();
        Core.bitwise_and(HSV, HSV, yellowMask, scaledThresh);

        //Convert the final yellowMask mat to RGB for camera stream
        Imgproc.cvtColor(yellowMask, output, Imgproc.COLOR_HSV2RGB);

        //Set the 3 sub-mats from the points above to split camera view into 3
        left = output.submat(new Rect(topLeftL, bottomRightL));
        middle = output.submat(new Rect(topLeftM, bottomRightM));
        right = output.submat(new Rect(topLeftR, bottomRightR));

        //Average the red and green (which make up yellow) and sum them to get an avg for the mat
        leftAvg = Core.mean(left).val[0] + Core.mean(left).val[1];
        middleAvg = Core.mean(middle).val[0] + Core.mean(left).val[1];
        rightAvg = Core.mean(right).val[0] + Core.mean(left).val[1];

        //Draw white rectangles bounded by the topLeft and bottomRight points on the camera view
        //Visible if camera stream is opened in init.
        Imgproc.rectangle(output, topLeftL, bottomRightL, WHITE, 1);
        Imgproc.rectangle(output, topLeftM, bottomRightM, WHITE, 1);
        Imgproc.rectangle(output, topLeftR, bottomRightR, WHITE, 1);

        //Logic for determining where the yellow object is
        //Camera is split into 3 as described before --> Left | Middle | Right
        //loc holds the position of the yellow object.-->  0  |    1   |  2
        //Get the max avg of the three mat areas
        max = Math.max(Math.max(leftAvg, rightAvg), middleAvg);
        //If max <= 7, it means no pole is successfully detected, the value is background interference
        //loc = -1 will signify an error of no yellow pole found
        if(max <= 7){
            loc = -1;
        }else{
            //If max  is left
            if(max == leftAvg){
                loc = 0;
                //If max is middle
            }else if(max == middleAvg) {
                loc = 1;
                //If max is right
            }else{
                loc = 2;
            }
        }
        //return the output mat (what will show up on camera stream)
        return output;
    }
}
