package org.firstinspires.ftc.teamcode.roverRuckus.other;


import android.os.Handler;
import android.support.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.ThreadPool;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCharacteristics;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraException;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraManager;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.internal.camera.WebcamExample;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.robotcore.internal.system.Misc;
import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.vision.UVC.UVCCamera;
import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.vision.UVC.WebcamExampleTesting;

import java.util.concurrent.Executor;
import java.util.concurrent.TimeUnit;

@TeleOp(name = "Webcam Test")
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
            new UVCCamera(cameraManager,cameraName).test(telemetry);


        }
    }

    @Override
    public void loop() {

    }


}
