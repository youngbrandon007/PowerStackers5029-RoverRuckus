package org.firstinspires.ftc.teamcode.roverRuckus.Robot_r2;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

@TeleOp(name="r2.PID",group = "r2")
public class PID_r2 extends Config{

    ArrayList times;
    ArrayList value;

    ElapsedTime time = new ElapsedTime();

    @Override
    public void init() {
        config(this);

        if(set.robotLiveEnabled) robotLive = new RobotLive();

        times = new ArrayList<String>();
        value = new ArrayList<String>();


        while (gyro.navxMicro.isCalibrating())  {
            telemetry.addData("gyro.status", "calibrating");
        }
        telemetry.addData("gyro.status", "ready");
    }

    @Override
    public void init_loop(){
        drive.P += (gamepad1.left_stick_y > .9) ? .001 : (gamepad1.left_stick_y < -.9) ? -.001 : 0.0;
        drive.D += (gamepad1.right_stick_y > .9) ? .001 : (gamepad1.right_stick_y < -.9) ? -.001 : 0.0;

        telemetry.addLine("P:" + drive.P + " I:" + drive.I + " D:" + drive.D);
        telemetry.addData("gyro.heading", gyro.getAngle());
        telemetry.update();
    }

    @Override
    public void start(){
        time.reset();
    }

    @Override
    public void loop() {
        times.add(String.valueOf(Math.round(time.milliseconds()/1000.0)));

        value.add(String.valueOf(Math.round(gyro.getAngle()/1000.0)*1000.0));//pid value


        if(set.robotLiveEnabled) {
            robotLive.data.addChartData("PID", times, value);
            robotLive.data.addStringData("Gyro", String.valueOf(gyro.getAngle()));
            robotLive.send();
        }
    }
}
