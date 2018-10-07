package org.firstinspires.ftc.teamcode.roverRuckus.sampleTesting;

import android.graphics.Bitmap;
import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.vision.UVC.UVCCamera;
import org.firstinspires.ftc.teamcode.RobotLiveTest.RobotLiveSendTemp;

import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;
import java.io.File;
import java.io.FileOutputStream;

import RobotLiveDataSending.RobotLiveSend;

@TeleOp(name = "UVC-Camera", group = "test")
public class test extends config implements UVCCamera.Callback {

    UVCCamera camera;
    Bitmap bm;

    @Override
    public void init() {
        config(this);
        camera = UVCCamera.getCamera(this);

        data = RobotLiveSendTemp.createNewRun("http://192.168.0.116");
    }

    @Override
    public void start() {
        camera.start();
    }

    @Override
    public void loop() {

        if(data == null){
            telemetry.addData("Error","data is null");

        }else {
            if (bm != null) {
                ByteArrayOutputStream bos = new ByteArrayOutputStream();
                bm.compress(Bitmap.CompressFormat.JPEG, 9 /*ignored for PNG*/, bos);
                byte[] bitmapdata = bos.toByteArray();
                ByteArrayInputStream bs = new ByteArrayInputStream(bitmapdata);


                data.addLiveImage(bs);
            }

            data.addStringData("Test", "Data is coming");

            RobotLiveSendTemp.send(data, "http://192.168.0.116");
        }
    }


    @Override
    public Bitmap onFrame(Bitmap bm) {
        this.bm = bm;
        telemetry.addLine("Image");
        return bm;
    }
}
