package org.firstinspires.ftc.teamcode.roverRuckus.Robot_r3;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

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

import static org.firstinspires.ftc.teamcode.roverRuckus.Robot_r3.Config_r3.AutoTasks.LAND;
import static org.firstinspires.ftc.teamcode.roverRuckus.Robot_r3.Config_r3.AutoTasks.SAMPLEPICTURE;
import static org.firstinspires.ftc.teamcode.roverRuckus.Robot_r3.Config_r3.AutoTasks.TAKEPICTURE;
import static org.firstinspires.ftc.teamcode.roverRuckus.Robot_r3.Config_r3.AutoTasks.WAITPICTURE;


@Autonomous(name = "r3.Auto", group = "r3")
public class Auto_r3 extends Config_r3 implements UVCCamera.Callback{

    //General
    ElapsedTime time = new ElapsedTime();
    AutoTasks task = AutoTasks.UNRATCHET;
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
        time.reset();
    }

    @Override
    public void loop() {
        //PID value
        drive.PIDoutput = 0.0;

        switch (task){
            case UNRATCHET:
                lift.extension.setPower(-1);
                lift.ratchetOn();
                if (time.milliseconds()>500){
                    task = LAND;
                    time.reset();
                    lift.bridge.setBridge2(90);
                    lift.extension.setPower(0);
                }
                break;
            case LAND:
                lift.extension.setPower(1);
                if (lift.extension.getEncoderPosition()>7500){
                    task = TAKEPICTURE;
                    lift.extension.setPower(0);
                }

                break;
            case TAKEPICTURE:
                //Get picture
                camera.start();
                time.reset();
                task = WAITPICTURE;
                break;
            case WAITPICTURE:
                if (time.milliseconds()>1000){
                    task = SAMPLEPICTURE;
                }
                break;
            case SAMPLEPICTURE:

                if(samplePos == 0){
                    samplePos = 2;
                }
                camera.stop();
                task = AutoTasks.SAMPLEDRIVE;
                drive.resetEncoders();
                break;
            case IDLE:
                break;
        }
//        tel.add("auto.status", task);
//        tel.add("drive.encoder-LF", drive.leftFront.getEncoderPosition());
//        tel.add("drive.encoder-RF", drive.rightFront.getEncoderPosition());
//        tel.add("drive.encoder-LB", drive.leftBack.getEncoderPosition());
//        tel.add("drive.encoder-RB", drive.rightBack.getEncoderPosition());
//        tel.update();

        telemetry.addData("sample.pos", samplePos);
        telemetry.addData("task", task);
    }


    @Override
    public void stop() {
//        camera.stop();
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
        Core.inRange(hsv, new Scalar(15, 100, 100), new Scalar(40, 255, 255), mask);

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

//        tel.add("sample.sample", samplePos);
//        tel.add("sample.location", locationX);
//        tel.add("sample.position", posX);
//        tel.add("sample.size", size);

//        if(task == WAITPICTURE)
//            task = SAMPLEPICTURE;
        if(set.saveImages){
//            PSVisionUtils.saveImageToFile(PSVisionUtils.matToBitmap(mask),"R2-frame", "/saved_images");
        }

//        camera.stop();
        return null;
    }


}
