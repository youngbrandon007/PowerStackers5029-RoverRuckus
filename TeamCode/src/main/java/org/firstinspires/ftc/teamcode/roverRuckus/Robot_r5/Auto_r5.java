package org.firstinspires.ftc.teamcode.roverRuckus.Robot_r5;

import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.path.heading.SplineInterpolator;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Paths.PathPoints;


@Autonomous(name = "r5.Auto", group = "r5")
public class Auto_r5 extends Config_r5 {

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

    //init
    @Override
    public void init() {
        //load data from auto app
        PathPoints.overridePoints();

        data = LoadConfig.getConfig();
        telemetry.addData("CONFIG", data);

        //init robot
        config(this);

        //load settings into variables
        startDepot = getSetting(2); //string formating to boolean
        drive.estimatedPosition = (startDepot) ? new Vector2d(55, 89) : new Vector2d(55, 55); //set trajecotory starting positions
        //gyro starting position
        gyro.cal = (startDepot) ? 135 : 225;
        gyro.cal += (getSetting(3)) ? 2: 0; //adjust value for hanging
        //load sampling settings
        sample = getSetting(4);
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
        taskTime.reset();
        //set starting action if hanging
        if(getSetting(3))
            task = Tasks.UNRATCHET;
        else
            task = Tasks.DELAY;
        //calculate delay
        delay = Double.valueOf(data.split(",")[1].split("=")[1]);

        task = Tasks.PRECYCLES;
    }

    private boolean getSetting(int index){
        return data.split(",")[index].split("=")[1].equals("true");
    }

    //loop state-make machine
    @Override
    public void loop() {
        //state switch statement
        switch (task) {

            case DELAY:
                break;
            case UNRATCHET:
                break;
            case LAND:
                break;
            case DRIVEBACK:
                break;
            case PICTURE:
                break;
            case PRETRAJECTORY:
                break;
            case TRAJECTORY:
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
                break;
        }
        //show pos
        telemetry.addData("pose", drive.getEstimatedPose());
    }

    private void next(Tasks nextTask){
        task = nextTask;
        taskTime.reset();
    }
}
