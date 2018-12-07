package org.firstinspires.ftc.teamcode.roverRuckus.Robot_r2;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.utils.vision.PSVisionUtils;
import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.vision.UVC.UVCCamera;
import org.opencv.android.OpenCVLoader;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

import static org.firstinspires.ftc.teamcode.roverRuckus.Robot_r2.Config.AutoTasks.LAND;
import static org.firstinspires.ftc.teamcode.roverRuckus.Robot_r2.Config.AutoTasks.SAMPLEPICTURE;
import static org.firstinspires.ftc.teamcode.roverRuckus.Robot_r2.Config.AutoTasks.TAKEPICTURE;
import static org.firstinspires.ftc.teamcode.roverRuckus.Robot_r2.Config.AutoTasks.WAITPICTURE;

@Autonomous(name = "r2.SampleTest", group = "r2")
public class SampleTest_r2 extends Config implements UVCCamera.Callback{

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
        if(set.useCamera) {
            camera.load(this);
            set.useCamera = (camera != null);
            if (set.useCamera) camera.start();
        }

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
//                telemetry.addData("Pos", samplePos);
//                telemetry.update();
    }


    @Override
    public void stop() {
        camera.stop();
    }

    @Override
    public Bitmap onFrame(Bitmap bm) {
        //Mask
        Mat input = new Mat();
//        Imgproc.f
        Mat hsv = new Mat();
        Bitmap bmp32 = bm.copy(Bitmap.Config.ARGB_8888, true);
        Utils.bitmapToMat(bmp32, input);
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);
        Mat mask = new Mat();
        Core.inRange(hsv, new Scalar(25, 100, 100), new Scalar(35, 255, 255), mask);

        //Contours
        List<MatOfPoint> contours = new ArrayList<MatOfPoint>();
        Imgproc.findContours(mask, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        double maxArea = 0;
        List<MatOfPoint> biggest = new ArrayList<>();
        int index = -1;
        Iterator<MatOfPoint> each = contours.iterator();
        while (each.hasNext()) {
            MatOfPoint wrapper = each.next();
            double area = Imgproc.contourArea(wrapper);
            if (area > maxArea) {
                maxArea = area;
                List<MatOfPoint> current = new ArrayList<>();
                current.add(wrapper);
                biggest = current;
            }
        }

        //@todo fix this so it only looks at the biggest contour
        //Centroid
        Moments mmnts = Imgproc.moments(mask, true);
        double locationX = mmnts.get_m10() / mmnts.get_m00();
        double posX = (input.width() / 2) - locationX;
        double size = maxArea;

        if(posX < con.sampleRange[0]){
            samplePos = 3;
        }else if(posX > con.sampleRange[0] && posX < con.sampleRange[1]){
            samplePos = 2;
        }else if(posX > con.sampleRange[1]){
            samplePos = 1;
        }

        telemetry.addData("sample.sample", samplePos);
        telemetry.addData("sample.location", locationX);
        telemetry.addData("sample.position", posX);
        telemetry.addData("sample.size", size);
        telemetry.update();

        if(set.saveImages){
            PSVisionUtils.saveImageToFile(bm,"R2-frame", "/saved_images");
        }

        return bm;
    }


}
