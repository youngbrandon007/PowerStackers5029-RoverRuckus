package org.firstinspires.ftc.teamcode.roverRuckus.Robot_r2;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "r2.AutoTasks", group = "r2")
public class Auto_r2 extends Config{

    ElapsedTime time = new ElapsedTime();
    boolean robotLiveEnabled = true;
    AutoTasks task = AutoTasks.LAND;
    int samplePos = 0;

    @Override
    public void init() {
        config(this);
        if(robotLiveEnabled) robotLive = new RobotLive();
    }

    @Override
    public void init_loop() {
        tel.update();
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

                task = AutoTasks.SAMPLEDRIVE;
                drive.resetEncoders();
                break;
            case SAMPLEDRIVE:
                drive.setMecanum(con.smapleAngle[samplePos], .2, true);

                if(drive.distanceTraveled() > con.sampleDis[samplePos]){
                    task = AutoTasks.SAMPLEDRIVEBACK;
                    drive.resetEncoders();
                }
                break;
            case SAMPLEDRIVEBACK:
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
        tel.update();
    }


    @Override
    public void stop() {

    }
}
