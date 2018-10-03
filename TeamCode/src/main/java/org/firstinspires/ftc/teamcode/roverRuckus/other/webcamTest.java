package org.firstinspires.ftc.teamcode.roverRuckus.other;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraManager;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.internal.camera.WebcamExample;
import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.vision.UVC.UVCCamera;
@TeleOp(name = "Webcam Test")
public class webcamTest extends OpMode{

    @Override
    public void init() {
        CameraManager cameraManager = ClassFactory.getInstance().getCameraManager();
        for (CameraName cameraName : cameraManager.getAllWebcams())
        {
            new UVCCamera(cameraManager, cameraName).test(telemetry);
        }
    }



    @Override
    public void loop() {

    }

}
