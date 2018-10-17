package org.firstinspires.ftc.teamcode.roverRuckus.Testing.other;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraManager;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;

//@TeleOp(name = "Webcam Test")
public class webcamTest extends OpMode{

    //final Executor threadPool = ThreadPool.newSingleThreadExecutor("UVCCamera");

    @Override
    public void init() {


    }

    @Override
    public void start() {

        CameraManager cameraManager = ClassFactory.getInstance().getCameraManager();
        for (CameraName cameraName : cameraManager.getAllWebcams())
        {
            //new UVCCamera(cameraManager,cameraName).start();


        }
    }

    @Override
    public void loop() {

    }


}
