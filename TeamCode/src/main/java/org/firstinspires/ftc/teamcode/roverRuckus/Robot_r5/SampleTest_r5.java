package org.firstinspires.ftc.teamcode.roverRuckus.Robot_r5;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.utils.vision.PSVisionUtils;
import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.vision.UVC.UVCCamera;
import org.opencv.android.OpenCVLoader;
import org.opencv.android.Utils;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

@Autonomous(name = "r5.SampleTest", group = "r4")

public class SampleTest_r5 extends Config_r5 implements UVCCamera.Callback{

    //General

    int samplePos = 0;

    //Camera
    static String opencvLoad = "";
    static {
        if (!OpenCVLoader.initDebug()) {
            opencvLoad = "Error Loading!";
        } else {
            opencvLoad = "Loaded Successfully!";
        }
    }

    @Override
    public void init() {
        config(this);
        lift.ratchetOff();
        camera.load(this);


    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        camera.start();
    }

    @Override
    public void loop() {

    }


    @Override
    public void stop() {
        camera.stop();
    }

    @Override
    public Bitmap onFrame(Bitmap bm) {
        //Mask
        Mat input = new Mat();
        Mat hsvLeft = new Mat();
        Mat hsvMiddle = new Mat();
        Bitmap bmp32 = bm.copy(Bitmap.Config.ARGB_8888, true);
        Utils.bitmapToMat(bmp32, input);
        Rect rectCrop = new Rect(40, 100 ,160 , 160);
        Mat leftMineral = input.submat(rectCrop);
        rectCrop = new Rect(440, 150 , 160, 120);
        Mat middleMineral = input.submat(rectCrop);
        PSVisionUtils.saveImageToFile(PSVisionUtils.matToBitmap(leftMineral),"R4-left", "/saved_images");
        PSVisionUtils.saveImageToFile(PSVisionUtils.matToBitmap(middleMineral),"R4-middle", "/saved_images");

        Imgproc.cvtColor(leftMineral, hsvLeft, Imgproc.COLOR_RGB2HSV);
        Imgproc.cvtColor(middleMineral, hsvMiddle, Imgproc.COLOR_RGB2HSV);
        double leftYellowArea = PSVisionUtils.hsvToTotalAreaInMask(hsvLeft,new Scalar(15, 100, 100), new Scalar(40, 255, 255),"leftY");
        double middleYellowArea = PSVisionUtils.hsvToTotalAreaInMask(hsvMiddle,new Scalar(15, 100, 100), new Scalar(40, 255, 255),"middleY");
//        double leftWhiteArea = PSVisionUtils.hsvToTotalAreaInMask(hsvLeft,new Scalar(0,0,180),new Scalar(180,20,255),"leftW");
//        double middleWhiteArea = PSVisionUtils.hsvToTotalAreaInMask(hsvMiddle, new Scalar(0,0,180),new Scalar(180,20,255),"middleW");
        telemetry.addData("leftYellow",leftYellowArea);
        telemetry.addData("middleYellow",middleYellowArea);

//        Core.inRange(hsvLeft, new Scalar(15, 100, 100), new Scalar(40, 255, 255), maskYellowLeft);
//        Core.inRange(hsvMiddle, new Scalar(15, 100, 100), new Scalar(40, 255, 255), maskYellowMiddle);
        //Contours
//        List<MatOfPoint> contours = new ArrayList<MatOfPoint>();
//        Imgproc.findContours(maskYellowLeft, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

//        double maxArea = 0;
//        List<MatOfPoint> biggest = new ArrayList<>();
//        int index = -1;
//        double totalAreaLeft =0;
//        Iterator<MatOfPoint> each = contours.iterator();
//        while (each.hasNext()) {
//            MatOfPoint wrapper = each.next();
//            double area = Imgproc.contourArea(wrapper);
//            totalAreaLeft += area;
//        }

        //@todo fix this so it only looks at the biggest contour
        //Centroid
//        Moments mmnts = Imgproc.moments(mask, true);
//        double locationX = mmnts.get_m10() / mmnts.get_m00();
//        double posX = (input.width() / 2) - locationX;
//        double size = maxArea;

//        if(posX < con.sampleRange[0]){
//            samplePos = 3;
//        }else if(posX > con.sampleRange[0] && posX < con.sampleRange[1]){
//            samplePos = 2;
//        }else if(posX > con.sampleRange[1]){
//            samplePos = 1;
//        }
//
        samplePos = ((leftYellowArea>1000)?1:((middleYellowArea>1000)?2:3));
        telemetry.addData("sample.sample", samplePos);
//        telemetry.addData("sample.location", locationX);
//        telemetry.addData("sample.position", posX);
//        telemetry.addData("sample.size", size);
        telemetry.update();

        return bm;
    }


}
