package org.firstinspires.ftc.teamcode.roverRuckus.Testing.cameraTesting;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.utils.vision.PSVisionUtils;
import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.vision.UVC.UVCCamera;

@TeleOp(name = "UVC Camera", group = "test")
public class UVCTesting extends config implements UVCCamera.Callback{

    ElapsedTime time = new ElapsedTime();
    UVCCamera camera;

    Bitmap send;

    @Override
    public void init() {
        config(this);


        camera = UVCCamera.getCamera(this);

        time.reset();

//        data = RobotLiveSend.createNewRun("http://192.168.200.174");
    }

    @Override
    public void start(){
        camera.start();
    }

    @Override
    public void loop() {
//       data.addLiveImage(send);
//
//       data.addStringData("Sent", "1");
//
//       telemetry.addLine(RobotLiveSend.send(data, "http://192.168.200.174"));
        if(send != null) {
            PSVisionUtils.saveImageToFile(send, "UVCcamera.jpg", "/saved_images");
        }
    }

    @Override
    public void stop(){
        camera.stop();
    }

    @Override
    public Bitmap onFrame(Bitmap bm) {
        telemetry.addData("UVC.FT",Math.round(time.milliseconds()));
        telemetry.addData("UVC.FPS",Math.round(1000/time.milliseconds()));
        time.reset();
        send = bm.copy(bm.getConfig(), true);

        return null;
    }
}
