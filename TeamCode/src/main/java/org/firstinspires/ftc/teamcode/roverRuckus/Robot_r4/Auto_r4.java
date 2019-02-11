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

    //data form auto app
    String data;

    //timer for trajectory
    ElapsedTime time = new ElapsedTime();

    //current state of auto
    Tasks task;

    // auto settings
    double delay;
    int samplePos = 0;
    boolean sample;
    boolean startDepot;

    //path object
    Trajectory trajectory;

    //states of auto
    enum Tasks {
        DELAY, UNRATCHET, LAND, DRIVEBACK,  PICTURE, PRETRAJECTORY, TRAJECTORY, IDLE
    }

    //load opencv for image capture
    static String opencvLoad = "";
    static {
        if (!OpenCVLoader.initDebug()) {
            opencvLoad = "Error Loading!";
        } else {
            opencvLoad = "Loaded Successfully!";
        }
    }

    //init
    @Override
    public void init() {
        //load data from auto app
        data = LoadConfig.getConfig();
        telemetry.addData("CONFIG", data);

        //init robot
        config(this);

        //load settings into variables
        startDepot = data.split(",")[2].split("=")[1].equals("true"); //string formating to boolean
        drive.estimatedPosition = (startDepot) ? new Vector2d(55, 89) : new Vector2d(55, 55); //set trajecotory starting positions
        //gyro starting position
        gyro.cal = (startDepot) ? 135 : 225;
        gyro.cal += (data.split(",")[3].split("=")[1].equals("true")) ? 2: 0; //adjust value for hanging
        //load camera
        camera.load(this);
        //load sampling settings
        sample = data.split(",")[4].split("=")[1].equals("true");
    }

    // wait for start loop
    @Override
    public void init_loop() {
        //show gyro value
        telemetry.addData("sensor.gyro", gyro.getHeading());
    }

    //start of auto
    @Override
    public void start() {
        //reset timer
        time.reset();
        //start camera if needed
        if(sample)
            camera.start();
        //set starting action if hanging
        if(data.split(",")[3].split("=")[1].equals("true"))
            task = Tasks.UNRATCHET;
        else
            task = Tasks.DELAY;
        //calculate delay
        delay = Double.valueOf(data.split(",")[1].split("=")[1]);
        //init bridge
        lift.bridge.setBridge2(20);
    }

    //loop state-make machine
    @Override
    public void loop() {
        //state-make switch statement
        switch (task) {
            case UNRATCHET:
                //if hanging unrachet motor
                lift.extension.setPower(-1);
                lift.ratchetOn();
                //unrachtet for .5 seconds
                if (time.milliseconds()>500){
                    //set next taks to LAND
                    task = task.LAND;
                    //reset timer for next task
                    time.reset();
                    //move bridge up
                    lift.bridge.setBridge2(90);
                    //turn off lift motor
                    lift.extension.setPower(0);
                }
                break;
            case LAND:
                //lower robot
                lift.extension.setPower(1);
                //wait for encoder to pass fully extended
                if (lift.extension.getEncoderPosition()<-8700){
                    //turn off lift
                    lift.extension.setPower(0);
                    //set next task
                    task = task.DRIVEBACK;
                    //reset timer
                    time.reset();
                }
                break;
            case DRIVEBACK:
                //drive into lander to straighten robot
                robot.drive.mecanum.setMecanum(Math.toRadians(270), .5, 0, 1);
                //done after .5 seconds
                if(time.milliseconds() > 500){
                    //stop drivetrain
                    robot.drive.stop();
                    //reset encoder saved values
                    //create save object
                    double[] rotations = new double[4];

                    //save encoder values
                    for (int i = 0; i < 4; i++) {
                        //read positions
                        int encoderPosition = drive.motors.get(i).getEncoderPosition();
                        //convert to radians
                        rotations[i] = drive.driveEncoderTicksToRadians(encoderPosition);
                    }
                    //set to position trakcing
                    drive.lastRotations = rotations;
                    //reset time
                    time.reset();
                    //reset calibaration of encoder
                    //  gyro.cal = gyro.getHeading();
                    //set next task
                    task = task.DELAY;
                }
            case DELAY:
                //set delay till sample starts
                //set collectory servo position
                collector.collectorRotate.setPosition(0.7);
                //wait for delay to complete
                if(time.milliseconds() >= delay * 1000){
                    //set next task based on sample settings
                    task = (sample) ? Tasks.PICTURE : Tasks.PRETRAJECTORY;
                }
                break;
            case PICTURE:
                //Wait for picture
                //see onFrame() function below
                break;
            case PRETRAJECTORY:
                //build trajectory based on smaple position, same function called by trajectory preview in app
                trajectory = PathGenerator.BuildPath(data, samplePos, DriveConstants_r4.BASE_CONSTRAINTS);

                //set follow trajectory
                drive.followTrajectory(trajectory);
                //reset timer
                time.reset();
                //set next task
                task = Tasks.TRAJECTORY;
                break;
            case TRAJECTORY:
                //update and follow trajectory
                //check if trajectory is still active
                if (drive.isFollowingTrajectory()) {
                    //show error
                    telemetry.addData("drive.error", drive.getFollowingError());
                    //check if trajectory is in sample position
                    if ((trajectory.get(time.seconds()).getY()==125)){ // (trajectory.get(time.seconds()).getX()==20)&&
                        //open marker release
                        drive.releaseMarker();
                        //set bridge positions
                        lift.bridge.setBridge2(180);
                    } else{
                        //if not in marker position
                        //close marker servo
                        drive.unreleaseMarker();
                    }
                    //show target position
                    telemetry.addData("targetPose",trajectory.get(time.seconds()));
                    //update telemetry
                    drive.update();
                } else {
                    //set robot to IDLE
                    task = Tasks.IDLE;
                }
                break;
            case IDLE:
                break;
        }
        //show pos
        telemetry.addData("pose", drive.getEstimatedPose());
    }

    @Override
    public Bitmap onFrame(Bitmap bm) {
        //if ready for picture process bitmap
        if(task == Tasks.PICTURE) {
            //create proccessing image objects
            Mat input = new Mat();
            Mat hsvLeft = new Mat();
            Mat hsvMiddle = new Mat();
            //convet image config for OpenCV
            Bitmap bmp32 = bm.copy(Bitmap.Config.ARGB_8888, true);
            Utils.bitmapToMat(bmp32, input);
            //crop section for left mineral
            Rect rectCrop = new Rect(40, 100, 160, 160);
            Mat leftMineral = input.submat(rectCrop);
            //crop seciton for middle mineral
            rectCrop = new Rect(440, 150, 160, 120);
            Mat middleMineral = input.submat(rectCrop);
            //save images for debugging (slows down program)
//            PSVisionUtils.saveImageToFile(PSVisionUtils.matToBitmap(leftMineral), "R4-left", "/saved_images");
//            PSVisionUtils.saveImageToFile(PSVisionUtils.matToBitmap(middleMineral), "R4-middle", "/saved_images");
            //convert color for masking (HSV is more accurate range identifcation than RGB)
            Imgproc.cvtColor(leftMineral, hsvLeft, Imgproc.COLOR_RGB2HSV);
            Imgproc.cvtColor(middleMineral, hsvMiddle, Imgproc.COLOR_RGB2HSV);
            //area of mask function
            //gold of left
            double leftYellowArea = PSVisionUtils.hsvToTotalAreaInMask(hsvLeft, new Scalar(15, 100, 100), new Scalar(40, 255, 255), "leftY");
            //gold of middle
            double middleYellowArea = PSVisionUtils.hsvToTotalAreaInMask(hsvMiddle, new Scalar(15, 100, 100), new Scalar(40, 255, 255), "middleY");
            //debug telemetry
            telemetry.addData("leftYellow", leftYellowArea);
            telemetry.addData("middleYellow", middleYellowArea);

            //determine position based on pixel area of gold in the left and middle minerals
            samplePos = ((leftYellowArea > 1000) ? 1 : ((middleYellowArea > 1000) ? 2 : 3));
            //telemetry for debugging
            telemetry.addData("sample.sample", samplePos);
            telemetry.update();

            //set next task
            task = Tasks.PRETRAJECTORY;
            //stop camera after proccessing
            camera.stop();
        }
        //don't save image
        return null;
    }
}
