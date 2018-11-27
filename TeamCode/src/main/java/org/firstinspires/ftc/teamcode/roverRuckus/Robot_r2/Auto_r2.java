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

@Autonomous(name = "r2.Auto", group = "r2")
public class Auto_r2 extends Config implements UVCCamera.Callback{


    ///////////////////////////////////----Run Settings----///////////////////////////////////////////
    static class set {
        static boolean doLanding = false;
        static boolean doSample = true;
        static boolean useCamera = true;

        //Real Match these value must be FALSE
        static boolean saveImages = false;
        static boolean robotLiveEnabled = true;
    }
    ///////////////////////////////////////////////////////////////////////////////////////////////////

    //General
    ElapsedTime time = new ElapsedTime();
    AutoTasks task = AutoTasks.LAND;
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
        tel.updateAll();
        tel.add("auto.status", "init");
        tel.updateAdd();

        config(this);
        tel.add("robotLive.enabled", set.robotLiveEnabled);
        if(set.robotLiveEnabled) {
            robotLive = new RobotLive();
            set.robotLiveEnabled = (robotLive != null);
            tel.add("robotLive.connected", set.robotLiveEnabled);
        }
        tel.updateAdd();

        tel.add("camera.enabled", set.useCamera);
        tel.add("camera.opencv", opencvLoad);
        if(set.useCamera) {
            camera.load(this);
            set.useCamera = (camera != null);
            if (set.useCamera) camera.start();
            tel.add("camera.status", (set.useCamera) ? "started" : "error");
        }
        tel.updateAdd();

        tel.add("auto.status", "init_loop");
        tel.updateAdd();
    }

    @Override
    public void init_loop() {
        if(gamepad1.y){
            camera.load(this);
            set.useCamera = (camera != null);
            if (set.useCamera) camera.start();
            tel.add("camera.status", (set.useCamera) ? "started" : "error");
        }

        tel.updateNoClear();
    }

    @Override
    public void start() {
        time.reset();
    }

    @Override
    public void loop() {
        //PID value
        drive.PIDoutput = 0.0;

        switch (task){
            case LAND:

                task = AutoTasks.SAMPLEPICTURE;
                break;
            case SAMPLEPICTURE:
                //Get picture

                task = (samplePos == 0) ? AutoTasks.DRIVEROTATETOWALL : AutoTasks.SAMPLEDRIVE;
                drive.resetEncoders();
                break;
            case SAMPLEDRIVE:
                drive.setMecanum(con.sampleAngle[samplePos], .2, true);

                if(drive.distanceTraveled() > con.sampleDis[samplePos]){
                    task = AutoTasks.SAMPLEDRIVEBACK;
                    drive.resetEncoders();
                }
                break;
            case SAMPLEDRIVEBACK:
                drive.setMecanum(270, .2, true);

                if(drive.distanceTraveled() > 4){
                    task = AutoTasks.DRIVEROTATETOWALL;
                    drive.resetEncoders();
                }
                break;
            case DRIVEROTATETOWALL:
                break;
            case DRIVETOWALL:
                break;
            case DRIVEROTATETODEPOT:
                break;
            case DRIVETODEPOT:
                break;
            case SAMPLESECONDDRIVE:
                break;
            case SAMPLESECONDDRIVEBACK:
                break;
            case CLAIM:
                break;
            case PARKDRIVEBACK:
                break;
        }
        tel.add("auto.status", task);
        tel.add("drive.encoder-LF", drive.leftFront.getEncoderPosition());
        tel.add("drive.encoder-RF", drive.rightFront.getEncoderPosition());
        tel.add("drive.encoder-LB", drive.leftBack.getEncoderPosition());
        tel.add("drive.encoder-RB", drive.rightBack.getEncoderPosition());
        tel.update();
    }


    @Override
    public void stop() {
        camera.stop();
    }

    @Override
    public Bitmap onFrame(Bitmap bm) {
        //Mask
        Mat input = new Mat();
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
            samplePos = 2;
        }else if(posX > con.sampleRange[0] && posX < con.sampleRange[1]){
            samplePos = 1;
        }else if(posX > con.sampleRange[1]){
            samplePos = 0;
        }

        tel.add("sample.sample", samplePos);
        tel.add("sample.location", locationX);
        tel.add("sample.position", posX);
        tel.add("sample.size", size);


        if(set.saveImages){
            PSVisionUtils.saveImageToFile(bm,"R2-frame", "/saved_images");
        }
        return null;
    }


}
