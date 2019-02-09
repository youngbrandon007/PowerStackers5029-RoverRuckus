package org.firstinspires.ftc.teamcode.roverRuckus.Robot_r4;

import android.graphics.Bitmap;

import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.utils.vision.PSVisionUtils;
import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.vision.UVC.UVCCamera;
import org.firstinspires.ftc.teamcode.roverRuckus.Robot_r4.Paths.PathGenerator;
import org.opencv.android.OpenCVLoader;
import org.opencv.android.Utils;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;



@Autonomous(name = "r4.Auto", group = "r4")
public class Auto_r4 extends Config_r4 implements UVCCamera.Callback {

    String data;

    ElapsedTime time = new ElapsedTime();

    Tasks task;

    double delay;
    int samplePos = 0;
    boolean sample;
    boolean startDepot;
    Trajectory trajectory;

    enum Tasks {
        DELAY, UNRATCHET, LAND, DRIVEBACK,  PICTURE, PRETRAJECTORY, TRAJECTORY, IDLE
    }

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
        data = LoadConfig.getConfig();
        telemetry.addData("CONFIG", data);

        config(this);

        startDepot = data.split(",")[2].split("=")[1].equals("true");
        drive.estimatedPosition = (startDepot) ? new Vector2d(55, 89) : new Vector2d(55, 55);
        gyro.cal = (startDepot) ? 135 : 225;
        gyro.cal += (data.split(",")[3].split("=")[1].equals("true")) ? 2: 0;
        camera.load(this);

        sample = data.split(",")[4].split("=")[1].equals("true");
    }

    @Override
    public void init_loop() {
        telemetry.addData("sensor.gyro", gyro.getHeading());
    }

    @Override
    public void start() {
        time.reset();
        if(sample)
            camera.start();
        if(data.split(",")[3].split("=")[1].equals("true"))
            task = Tasks.UNRATCHET;
        else
            task = Tasks.DELAY;
        delay = Double.valueOf(data.split(",")[1].split("=")[1]);
        lift.bridge.setBridge2(20);
    }

    @Override
    public void loop() {
        switch (task) {
            case UNRATCHET:
                lift.extension.setPower(-1);
                lift.ratchetOn();
                if (time.milliseconds()>500){
                    task = task.LAND;
                    time.reset();
                    lift.bridge.setBridge2(90);
                    lift.extension.setPower(0);
                }
                break;
            case LAND:
                lift.extension.setPower(1);
                if (lift.extension.getEncoderPosition()<-8700){
                    lift.extension.setPower(0);
                    task = task.DRIVEBACK;
                    time.reset();
//                    drive.rightFront.setupEncoder();
//                    drive.rightBack.setupEncoder();
//                    drive.leftBack.setupEncoder();
//                    drive.leftFront.setupEncoder();
                    //gyro.cal = gyro.getHeading();
                }
                break;
            case DRIVEBACK:
                robot.drive.mecanum.setMecanum(Math.toRadians(270), .5, 0, 1);
                if(time.milliseconds() > 500){
                    robot.drive.stop();
                    double[] rotations = new double[4];

                    for (int i = 0; i < 4; i++) {
                        int encoderPosition = drive.motors.get(i).getEncoderPosition();
                        rotations[i] = drive.driveEncoderTicksToRadians(encoderPosition);

                    }
                    drive.lastRotations = rotations;
                    time.reset();
                    //  gyro.cal = gyro.getHeading();
                    task = task.DELAY;
                }
            case DELAY:
                collector.collectorRotate.setPosition(0.7);
                if(time.milliseconds() >= delay * 1000){
                    task = (sample) ? Tasks.PICTURE : Tasks.PRETRAJECTORY;
                }
                break;
            case PICTURE:
                //Wait for picture
                break;
            case PRETRAJECTORY:

                trajectory = PathGenerator.BuildPath(data, samplePos, DriveConstants_r4.BASE_CONSTRAINTS);

                drive.followTrajectory(trajectory);
                time.reset();

                task = Tasks.TRAJECTORY;
                break;
            case TRAJECTORY:

                if (drive.isFollowingTrajectory()) {
                    telemetry.addData("drive.error", drive.getFollowingError());
                    if ((trajectory.get(time.seconds()).getY()==125)){ // (trajectory.get(time.seconds()).getX()==20)&&
                        drive.releaseMarker();
                        lift.bridge.setBridge2(180);
                    }
                    else{
                        drive.unreleaseMarker();
                    }
                    telemetry.addData("targetPose",trajectory.get(time.seconds()));
                    drive.update();
                } else {
                    task = Tasks.IDLE;
                }
                break;
            case IDLE:
                break;
        }

        telemetry.addData("pose", drive.getEstimatedPose());
    }

    @Override
    public Bitmap onFrame(Bitmap bm) {
        if(task == Tasks.PICTURE) {
            Mat input = new Mat();
            Mat hsvLeft = new Mat();
            Mat hsvMiddle = new Mat();
            Bitmap bmp32 = bm.copy(Bitmap.Config.ARGB_8888, true);
            Utils.bitmapToMat(bmp32, input);
            Rect rectCrop = new Rect(40, 100, 160, 160);
            Mat leftMineral = input.submat(rectCrop);
            rectCrop = new Rect(440, 150, 160, 120);
            Mat middleMineral = input.submat(rectCrop);
//            PSVisionUtils.saveImageToFile(PSVisionUtils.matToBitmap(leftMineral), "R4-left", "/saved_images");
//            PSVisionUtils.saveImageToFile(PSVisionUtils.matToBitmap(middleMineral), "R4-middle", "/saved_images");
            Imgproc.cvtColor(leftMineral, hsvLeft, Imgproc.COLOR_RGB2HSV);
            Imgproc.cvtColor(middleMineral, hsvMiddle, Imgproc.COLOR_RGB2HSV);
            double leftYellowArea = PSVisionUtils.hsvToTotalAreaInMask(hsvLeft, new Scalar(15, 100, 100), new Scalar(40, 255, 255), "leftY");
            double middleYellowArea = PSVisionUtils.hsvToTotalAreaInMask(hsvMiddle, new Scalar(15, 100, 100), new Scalar(40, 255, 255), "middleY");
            telemetry.addData("leftYellow", leftYellowArea);
            telemetry.addData("middleYellow", middleYellowArea);

            samplePos = ((leftYellowArea > 1000) ? 1 : ((middleYellowArea > 1000) ? 2 : 3));
            telemetry.addData("sample.sample", samplePos);
            telemetry.update();

            task = Tasks.PRETRAJECTORY;
            camera.stop();
        }
        return null;
    }
}
