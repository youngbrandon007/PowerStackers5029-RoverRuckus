package org.firstinspires.ftc.teamcode.roverRuckus.Testing.sampleTesting;

import android.graphics.Bitmap;
import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.utils.vision.PSVisionUtils;
import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.vision.UVC.UVCCamera;
import org.firstinspires.ftc.teamcode.RobotLive.RobotLiveSend;
import org.opencv.android.OpenCVLoader;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.io.File;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

//import RobotLiveDataSending.RobotLiveSend;

@TeleOp(name = "sampling", group = "test")
@Disabled
public class test extends config implements UVCCamera.Callback {


    UVCCamera camera;
    Bitmap bm;
    int sampPos = 1; //0 = Left, 1 = center
    int sampleNow;
    double angle;
    boolean save = false;


    static String load = "";

    static {
        if (!OpenCVLoader.initDebug()) {
            load = "Error Loading!";
        } else {
            load = "Loaded Successfully!";
        }
    }

    @Override
    public void init() {
        angle = Math.PI/2;
        config(this);
        camera = UVCCamera.getCamera(this);

        telemetry.addLine(load);
        data = RobotLiveSend.createNewRun("http://192.168.200.113");
    }

    @Override
    public void start() {
        camera.start();
    }

    @Override
    public void loop() {
        if (gamepad1.a){
            switch (sampleNow){
                case 0:
                    angle = Math.toRadians(120);
                    break;
                case 1:
                    angle = Math.toRadians(90);
                    break;
                case 2:
                    angle = Math.toRadians(60);
                    break;
            }
            robot.drive.mecanum.setMecanum(angle,0.3, 0, 1);
        } else {
            robot.drive.mecanum.updateMecanum(gamepad1,1);
            sampleNow = sampPos;
        }
        if(data == null){
            telemetry.addData("Error","data is null");

        }else {
            try {
                if (bm != null) {
//                    ByteArrayOutputStream bos = new ByteArrayOutputStream();
//                    bm.compress(Bitmap.CompressFormat.JPEG, 100 /*ignored for PNG*/, bos);
//                    byte[] bitmapdata = bos.toByteArray();
//                    InputStream bs = new ByteArrayInputStream(bitmapdata);
//
//
//                    data.addLiveImage(bs);
                    String root = Environment.getExternalStorageDirectory().getAbsolutePath();
                    File myDir = new File(root + "/saved_images");
                    myDir.mkdirs();

                    String fname = "image_before" + ".jpg";
                    File file = new File(myDir, fname);

                    //data.addLiveImage(file);

                } else telemetry.addData("Error", "image is null");

                data.addStringData("Test", "Data is coming");

                RobotLiveSend.send(data, "http://192.168.200.113");
                save = true;
            }catch(Exception e){
                telemetry.addData("Error",e);
            }
        }
    }


    @Override
    public Bitmap onFrame(Bitmap bm) {
        this.bm = bm;
        Mat input = new Mat();
        Mat hsv = new Mat();
        Bitmap bmp32 = bm.copy(Bitmap.Config.ARGB_8888, true);
        Utils.bitmapToMat(bmp32, input);
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);
        Mat mask = new Mat();
        Core.inRange(hsv, new Scalar(25, 100, 100), new Scalar(35, 255, 255), mask);
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

        //Centroid setup
        Moments mmnts = Imgproc.moments(mask, true);
        double locationX = mmnts.get_m10() / mmnts.get_m00();
        double posX = (input.width() / 2) - locationX;
        double size = maxArea;

        telemetry.addData("Location", locationX);
        telemetry.addData("Position", posX);
        telemetry.addData("Size", size);

        if(posX < -100){
            sampPos = 2;
        }else if(posX > -100 && posX < 100){
            sampPos = 1;
        }else if(posX > 100){
            sampPos = 0;
        }

        telemetry.addData("Sample", sampPos);
        if(save) {
            PSVisionUtils.saveImageToFile(bm,"image_before", "/saved_images");
            save = false;
            return PSVisionUtils.matToBitmap(mask);
        }

        return null;
//        return bm;
    }
}
