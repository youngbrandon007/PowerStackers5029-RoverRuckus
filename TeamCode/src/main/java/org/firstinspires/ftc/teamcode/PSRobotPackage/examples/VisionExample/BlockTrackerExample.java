package org.firstinspires.ftc.teamcode.PSRobotPackage.examples.VisionExample;

import org.firstinspires.ftc.teamcode.PSRobotPackage.lib.Vision.PSCamera;
import org.firstinspires.ftc.teamcode.PSRobotPackage.lib.Vision.PSOverlay;
import org.firstinspires.ftc.teamcode.PSRobotPackage.lib.Vision.PSTracker;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

public class BlockTrackerExample extends PSTracker {
    private Mat hsv,mask,bgr, gray;
    public static final int YELLOW_HSV_HUE_LOW = 20;
    public static final int YELLOW_HSV_SAT_LOW = 50;
    public static final int YELLOW_HSV_VAL_LOW = 20;

    public static final int YELLOW_HSV_HUE_HIGH = 50;
    public static final int YELLOW_HSV_SAT_HIGH = 100;
    public static final int YELLOW_HSV_VAL_HIGH = 100;

    Scalar HSVLOW = new Scalar(YELLOW_HSV_HUE_LOW,YELLOW_HSV_SAT_LOW, YELLOW_HSV_VAL_LOW);
    Scalar HSVHIGH = new Scalar(YELLOW_HSV_HUE_HIGH,YELLOW_HSV_SAT_HIGH,YELLOW_HSV_VAL_HIGH);


    @Override
    public void init(PSCamera camera) {
        hsv = new Mat();
        mask = new Mat();
        bgr = new Mat();
        gray = new Mat();
//        hsv = new Mat();

    }

    @Override
    public void processFrame(Mat frame, double timestamp) {
        //Formating
        Imgproc.cvtColor(frame, bgr, Imgproc.COLOR_RGB2BGR);
        Imgproc.cvtColor(frame,gray,Imgproc.COLOR_RGB2GRAY);

        //Blurring
        Imgproc.GaussianBlur(bgr,bgr,new Size(5,5),0);
        Imgproc.GaussianBlur(gray,gray,new Size(5,5),0);

        addIntermediate("bgrblur",bgr);
        addIntermediate("grayblur",gray);

        //Masking
        Imgproc.cvtColor(bgr,hsv,Imgproc.COLOR_BGR2HSV);
        Core.inRange(hsv,HSVLOW,HSVHIGH,mask);

        //Canny edge detection
        Imgproc.Canny(gray,gray,1,3,3,false);

        addIntermediate("edges",gray);
        addIntermediate("mask",mask);

    }

    @Override
    public void drawOverlay(PSOverlay overlay, int imageWidth, int imageHeight, boolean debug) {

    }
}
