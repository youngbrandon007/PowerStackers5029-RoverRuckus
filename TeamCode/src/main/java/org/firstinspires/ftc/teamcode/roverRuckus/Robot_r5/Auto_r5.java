package org.firstinspires.ftc.teamcode.roverRuckus.Robot_r5;

import android.graphics.Bitmap;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.path.heading.SplineInterpolator;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.utils.vision.PSVisionUtils;
import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.vision.UVC.UVCCamera;
import org.firstinspires.ftc.teamcode.Paths.PathGenerator;
import org.firstinspires.ftc.teamcode.Paths.PathPoints;
import org.firstinspires.ftc.teamcode.roverRuckus.Robot_r4.Auto_r4;
import org.firstinspires.ftc.teamcode.roverRuckus.Robot_r5.Transition.AutoTransitioner;
import org.opencv.android.OpenCVLoader;
import org.opencv.android.Utils;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;


@Autonomous(name = "r5.Auto", group = "r5")
public class Auto_r5 extends Config_r5 implements UVCCamera.Callback {

    //data form auto app
    String data;

    //timer for trajectory
    ElapsedTime time = new ElapsedTime();
    ElapsedTime taskTime = new ElapsedTime();

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
        DELAY, UNRATCHET, LAND, DRIVEBACK,  PICTURE, PRETRAJECTORY, TRAJECTORY, PRECYCLES, CYCLESDRIVEFORWARD, CYCLESCOLLECT, CYCLESDRIVEBACK, CYCLESPLACE,IDLE
    }


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
        PathPoints.overridePoints();

        data = LoadConfig.getConfig();

        //init robot
        config(this);

        //load settings into variables
        startDepot = getSetting(2); //string formating to boolean
        //drive.calibrationPosition = (startDepot) ? new Vector2d(55, 89) : new Vector2d(57, 57); //set trajecotory starting positions
        //gyro starting position
        gyro.cal = (startDepot) ? 135 : 225;
        drive.trackerWheels.setPoseEstimate((startDepot) ? new Pose2d(57, -87, Math.toRadians(225)) : new Pose2d(57, -57, Math.toRadians(135)));
        //gyro.cal += (getSetting(3)) ? 2: 0; //adjust value for hanging
        //load sampling settings
        sample = getSetting(4);

        camera.load(this);

        trajectory = PathGenerator.BuildPath(data, 2, DriveConstants_r5.BASE_CONSTRAINTS);
        lift.bridge.canopy.setPosition(1);

        drive.unreleaseMarker();
        //lights.theatre();
    }

    // wait for start loop
    @Override
    public void init_loop() {
        //show gyro value
        telemetry.addData("sensor.gyro", gyro.getHeading());
        telemetry.addData("CONFIG", data);


        lift.extension.setPower((gamepad1.dpad_up) ? -1.0 : (gamepad1.dpad_down) ? 1.0 : 0.0);
    }

    //start of auto
    @Override
    public void start() {
        //reset timer
        time.reset();
        //set starting action if hanging
        if(getSetting(3))
            task = Tasks.UNRATCHET;
        else
            task = Tasks.DELAY;
        //calculate delay
        delay = Double.valueOf(data.split(",")[1].split("=")[1]);

        camera.start();

        if(getSetting(3)){
            next(Tasks.UNRATCHET);
        }else{
            next(Tasks.PICTURE);
        }

        collector.collectorRotate.setPosition(1.0);

        AutoTransitioner.transitionOnStop(this, "r5.Tele");
    }

    private boolean getSetting(int index){
        return data.split(",")[index].split("=")[1].equals("true");
    }

    //loop state-make machine
    @Override
    public void loop() {
        //state switch statement
        switch (task) {
            case UNRATCHET:
                next(Tasks.LAND);
                break;
            case LAND:
                //lower robot
                lift.extension.setPower(-1);
                //wait for encoder to pass fully extended
                if (lift.extension.getEncoderPosition()<lift.liftExtendTickCount){
                    //turn off lift
                    lift.extension.setPower(0);
                    //set next task
                    next(task.DRIVEBACK);
                }
                break;
            case DRIVEBACK:
                //drive into lander to straighten robot
                robot.drive.mecanum.setMecanum(Math.toRadians(270), .5, 0, 1);
                //done after .5 seconds
                if(time.milliseconds() > 500){
                    //stop drivetrain
                    robot.drive.stop();
                    drive.trackerWheels.setPoseEstimate((startDepot) ? new Pose2d(57, -87, Math.toRadians(225)) : new Pose2d(57, -57, Math.toRadians(135)));
                    next(task.DELAY);
                }
                break;
            case DELAY:

                if(taskTime.seconds() > delay){
                    next(Tasks.PICTURE);
                }
                break;
            case PICTURE:
                break;
            case PRETRAJECTORY:
                trajectory = PathGenerator.BuildPath(data, samplePos, DriveConstants_r5.BASE_CONSTRAINTS);

                drive.followTrajectory(trajectory);

                next(Tasks.TRAJECTORY);
                break;
            case TRAJECTORY:
                if(drive.isFollowingTrajectory()){
                    drive.update();
                    Pose2d curTarget = trajectory.get(taskTime.seconds());
                    if(curTarget.getX() < 24 && curTarget.getY() > 115&&!startDepot){

                            drive.releaseMarker();

                    }else if(curTarget.getX() < 18 && curTarget.getY() > 115&&startDepot&&gyro.getHeading()<300&&gyro.getHeading()>260){
                        drive.releaseMarker();
                    }

                    else{
                        drive.unreleaseMarker();
                    }
                }else{
                    robot.drive.stop();
                    next(Tasks.IDLE);
                }
                break;
            case PRECYCLES:
               trajectory = new TrajectoryBuilder(drive.getEstimatedPose(), DriveConstants_r5.BASE_CONSTRAINTS)
                        .lineTo(new Vector2d(38,38), new SplineInterpolator(drive.getEstimatedPose().getHeading(), Math.toRadians(225)))
                        .build();
                drive.followTrajectory(trajectory);
                next(Tasks.CYCLESDRIVEFORWARD);
                break;
            case CYCLESDRIVEFORWARD:
                if (drive.isFollowingTrajectory()) {
                    drive.update();
                }else{
                    next(Tasks.CYCLESCOLLECT);
                }
                break;
            case CYCLESCOLLECT:
                collector.shooterLeft.setPower(-1.0);
                collector.shooterRight.setPower(1.0);

                if(taskTime.seconds() > 2.5){ //todo fill in with collecting code
                    collector.shooterLeft.setPower(0.0);
                    collector.shooterRight.setPower(0.0);
                    if(time.seconds() < 27) {
                        trajectory = new TrajectoryBuilder(drive.getEstimatedPose(), DriveConstants_r5.BASE_CONSTRAINTS)
                                .lineTo(new Vector2d(48, 48), new SplineInterpolator(drive.getEstimatedPose().getHeading(), Math.toRadians(225)))
                                .build();
                        drive.followTrajectory(trajectory);
                        next(Tasks.CYCLESDRIVEBACK);
                    }else{
                        next(Tasks.IDLE);
                    }
                }
                break;
            case CYCLESDRIVEBACK:
                if (drive.isFollowingTrajectory()) {
                    drive.update();
                }else{
                    next(Tasks.CYCLESPLACE);
                }
                break;
            case CYCLESPLACE:
                lift.bridge.doorServo.setPosition(lift.dropNormal);
                if(taskTime.seconds() > 1){ //todo finish placing
                    lift.bridge.doorServo.setPosition(lift.dropInit);
                    next(Tasks.PRECYCLES);
                }
                break;
            case IDLE:
                if(collector.extension.getEncoderPosition() > -2000&&taskTime.seconds()<4){
                    collector.extension.setPower(-1);
                }else if(collector.extension.getEncoderPosition() < -1000&&taskTime.seconds()<8&&taskTime.seconds()>4){
                    collector.extension.setPower(1);
                }
                else{
                    collector.extension.setPower(0);
                }
                if (lift.extension.getEncoderPosition()>-13300){
                    lift.extension.setPower(-1);
                }else{
                    lift.extension.setPower(0);
                }
                if(taskTime.seconds() >4 && taskTime.seconds()<8){
                    lift.bridge.openBridge();
                    lift.bridge.doorServo.setPosition(.8);
                    lift.bridge.canopy.setPosition(1.0);
                }else{
                    lift.bridge.stopBridge();
                }
                if(taskTime.seconds() > 7 && taskTime.seconds() < 8){
                    lift.bridge.canopy.setPosition(0.0);
                }
                break;
        }
        //show pos
        telemetry.addData("Error", drive.getFollowingError());
        telemetry.addData("pose", drive.getEstimatedPose());
        telemetry.addData("Target", trajectory.get(taskTime.milliseconds()));
        telemetry.addData("Sample", samplePos);
        telemetry.addData("Est. Encoder", collector.extension.getEncoderPosition());
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
            Rect rectCrop = new Rect(0, 120 ,160 , 120);
            Mat leftMineral = input.submat(rectCrop);
            //crop seciton for middle mineral
            rectCrop = new Rect(320, 120 , 160, 120);
            Mat middleMineral = input.submat(rectCrop);
            //save images for debugging (slows down program)
            //PSVisionUtils.saveImageToFile(PSVisionUtils.matToBitmap(leftMineral), "R4-left", "/saved_images");
            //PSVisionUtils.saveImageToFile(PSVisionUtils.matToBitmap(middleMineral), "R4-middle", "/saved_images");
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
            samplePos = ((leftYellowArea > middleYellowArea && leftYellowArea > 200) ? 1 : ((middleYellowArea > 200) ? 2 : 3));
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

    @Override
    public void stop(){
        savePosition();
    }

    private void next(Tasks nextTask){
        task = nextTask;
        taskTime.reset();
    }
}
