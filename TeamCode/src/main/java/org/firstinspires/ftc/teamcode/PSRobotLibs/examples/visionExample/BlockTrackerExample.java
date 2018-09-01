package org.firstinspires.ftc.teamcode.PSRobotLibs.examples.visionExample;

import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.vision.PSCamera;
import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.vision.PSOverlay;
import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.vision.PSTracker;
import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.utils.vision.PSVisionUtils;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

public class BlockTrackerExample extends PSTracker {
    private Mat hsv, hsvMask, bgr;
    public static final int YELLOW_HSV_HUE_LOW = 20;
    public static final int YELLOW_HSV_SAT_LOW = 50;
    public static final int YELLOW_HSV_VAL_LOW = 20;

    public static final int YELLOW_HSV_HUE_HIGH = 50;
    public static final int YELLOW_HSV_SAT_HIGH = 100;
    public static final int YELLOW_HSV_VAL_HIGH = 100;

    Scalar HSVLOW = new Scalar(YELLOW_HSV_HUE_LOW, YELLOW_HSV_SAT_LOW, YELLOW_HSV_VAL_LOW);
    Scalar HSVHIGH = new Scalar(YELLOW_HSV_HUE_HIGH, YELLOW_HSV_SAT_HIGH, YELLOW_HSV_VAL_HIGH);

    private Mat hsvMaskOpen, hsvMaskClose, hsvOpenKernel, hsvCloseKernel;
    private int hsvOpenKernelWidth, hsvOpenKernelHeight, hsvCloseKernelWidth, hsvCloseKernelHeight;


    public static int HSV_OPEN_KERNEL_WIDTH = 5;
    public static int HSV_OPEN_KERNEL_HEIGHT = 5;
    public static int HSV_CLOSE_KERNEL_WIDTH = 3;
    public static int HSV_CLOSE_KERNEL_HEIGHT = 31;

    @Override
    public void init(PSCamera camera) {
        hsv = new Mat();
        hsvMask = new Mat();
        bgr = new Mat();
        hsvMaskOpen = new Mat();
        hsvMaskClose = new Mat();
        hsvOpenKernel = new Mat();
        hsvCloseKernel = new Mat();
    }

    @Override
    public void processFrame(Mat frame, double timestamp) {
        PSVisionUtils.refreshKernel(hsvOpenKernel,hsvOpenKernelHeight,hsvOpenKernelWidth,HSV_OPEN_KERNEL_HEIGHT,HSV_OPEN_KERNEL_WIDTH);
        PSVisionUtils.refreshKernel(hsvCloseKernel,hsvCloseKernelHeight,hsvCloseKernelWidth,HSV_CLOSE_KERNEL_HEIGHT,HSV_CLOSE_KERNEL_WIDTH);
        //Color Conversion
        Imgproc.cvtColor(frame, bgr, Imgproc.COLOR_RGB2BGR);
        //Blurring
        Imgproc.GaussianBlur(bgr, bgr, new Size(5, 5), 0);
        addIntermediate("bgrblur", bgr);

        //Masking
        Imgproc.cvtColor(bgr, hsv, Imgproc.COLOR_BGR2HSV);
        Core.inRange(hsv, HSVLOW, HSVHIGH, hsvMask);
        addIntermediate("hsvMask", hsvMask);

        Imgproc.morphologyEx(hsvMask, hsvMaskOpen,Imgproc.MORPH_OPEN,hsvOpenKernel);
        addIntermediate("hsvMaskOpen", hsvMaskOpen);
        Imgproc.morphologyEx(hsvMaskOpen, hsvMaskClose,Imgproc.MORPH_CLOSE,hsvCloseKernel);
        addIntermediate("hsvMaskClose", hsvMaskClose);
    }

    @Override
    public void drawOverlay(PSOverlay overlay, int imageWidth, int imageHeight, boolean debug) {
        Scalar color = new Scalar(0,255,0);
        List<MatOfPoint> contours = new ArrayList<MatOfPoint>();
        Imgproc.findContours(hsvMaskClose, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        for (MatOfPoint current : contours) {
            double area = Imgproc.contourArea(current);
            overlay.strokeContour(current, color,3);
            overlay.putText(""+area, PSOverlay.TextAlign.CENTER,PSVisionUtils.maskCentroid(current),color,10);
        }


    }


}
