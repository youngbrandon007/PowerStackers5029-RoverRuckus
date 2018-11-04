package org.firstinspires.ftc.teamcode.roverRuckus.Testing.cameraTesting;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.vision.UVC.UVCCamera;
import org.firstinspires.ftc.teamcode.RobotLive.RobotLiveSend;

import java.util.ArrayList;

@TeleOp(name = "Robot Live", group = "test")
public class RobotLiveTesting extends config implements UVCCamera.Callback{

    ElapsedTime time = new ElapsedTime();
    ElapsedTime t = new ElapsedTime();
    UVCCamera camera;

    Bitmap send;

    @Override
    public void init() {
        config(this);


        camera = UVCCamera.getCamera(this);

        time.reset();
        t.reset();
        data = RobotLiveSend.createNewRun("http://192.168.200.113");
    }

    @Override
    public void start(){
        camera.start();
    }

    @Override
    public void loop() {
        if(gamepad1.b) {
            data.addLiveImage(send);
        }

       data.addStringData("a", (gamepad1.a) ? "True":"False");

       ArrayList<Double> x = new ArrayList<>();
       ArrayList<Double> y = new ArrayList<>();

       x.add(t.milliseconds()/1000.0);
       y.add((double) gamepad1.left_stick_y);


       telemetry.addLine(t.milliseconds() + "->" + x.get(0));
       telemetry.addLine(gamepad1.left_stick_y + "->" + y.get(0));

       data.addChartDouble("gamepad",x, y);

       telemetry.addLine(RobotLiveSend.send(data, "http://192.168.200.113"));

//        if(send != null) {
//            PSVisionUtils.saveImageToFile(send, "UVCcamera.jpg", "/saved_images");
//        }
    }

    @Override
    public void stop(){
        camera.stop();
    }

    @Override
    public Bitmap onFrame(Bitmap bm) {
        //telemetry.addData("UVC.FT",Math.round(time.milliseconds()));
        //telemetry.addData("UVC.FPS",Math.round(1000/time.milliseconds()));
        time.reset();
        send = bm.copy(bm.getConfig(), true);

        return null;
    }
}

