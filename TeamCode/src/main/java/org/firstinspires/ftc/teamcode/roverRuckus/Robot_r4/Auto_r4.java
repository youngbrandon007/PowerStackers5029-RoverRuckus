package org.firstinspires.ftc.teamcode.roverRuckus.Robot_r4;

import android.graphics.Bitmap;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.vision.UVC.UVCCamera;
import org.firstinspires.ftc.teamcode.roverRuckus.Robot_r4.Paths.PathGenerator;
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

@Autonomous(name = "r4.Auto", group = "r4")
public class Auto_r4 extends Config_r4 implements UVCCamera.Callback {

    String data;

    ElapsedTime time = new ElapsedTime();
    private double cal = 0;

    private Pose2d estimatedPose = new Pose2d(0, 0, 0);

    Tasks task = Tasks.LAND;

    int samplePos = 0;
    int pictureComplete = 0;

    enum Tasks {
        LAND, PICTURE, TRAJECTORY, IDLE
    }

    @Override
    public void init() {
        data = LoadConfig.getConfig();
        telemetry.addData("CONFIG", data);

        config(this);
    }

    @Override
    public void init_loop() {
        telemetry.addData("sensor.gyro", gyro.getHeading());
    }

    @Override
    public void start() {
        time.reset();
    }

    @Override
    public void loop() {
        switch (task) {
            case LAND:

                task = Tasks.PICTURE;
                break;
            case PICTURE:
                if (data.split(",")[4].split("=")[1].equals("true")) {

                } else {
                    pictureComplete = 101;
                }


                if (pictureComplete == 101) {
                    task = Tasks.TRAJECTORY;

                    Trajectory trajectory = PathGenerator.BuildPath(data, samplePos, DriveConstants_r4.BASE_CONSTRAINTS);

                    drive.followTrajectory(trajectory);
                    time.reset();
                }
                break;
            case TRAJECTORY:

                if (drive.isFollowingTrajectory()) {
                    telemetry.addData("drive.error", drive.getFollowingError());
                    drive.update();
                } else {
                    task = Tasks.IDLE;
                }
                break;
            case IDLE:
                break;
        }
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

        //todo test this code, it should create a mask with only the biggest contour
        Mat mask2 = new Mat(mask.rows(), mask.cols(), mask.type(), Scalar.all(0));
        Imgproc.drawContours(mask, biggest, 0, new Scalar(1), Core.FILLED);


        //@todo fix this so it only looks at the biggest contour
        //Centroid
        Moments mmnts = Imgproc.moments(mask, true);
        double locationX = mmnts.get_m10() / mmnts.get_m00();
        double posX = (input.width() / 2) - locationX;
        double size = maxArea;
        if (posX < Config_r4.con.sampleRange[0]) {
            samplePos = 3;
        } else if (posX > Config_r4.con.sampleRange[0] && posX < Config_r4.con.sampleRange[1]) {
            samplePos = 2;
        } else if (posX > Config_r4.con.sampleRange[1]) {
            samplePos = 1;
        }

//        camera.stop();
        return null;
    }
}
