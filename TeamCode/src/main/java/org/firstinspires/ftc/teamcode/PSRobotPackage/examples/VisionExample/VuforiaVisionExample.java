package org.firstinspires.ftc.teamcode.PSRobotPackage.examples.VisionExample;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.PSRobotPackage.lib.PSEnum;
import org.firstinspires.ftc.teamcode.PSRobotPackage.lib.Vision.PSVuforiaCamera;

public class VuforiaVisionExample extends OpMode {
    PSVuforiaCamera vuforiaCamera = new PSVuforiaCamera(PSEnum.CameraDirection.BACK);
    @Override
    public void init() {
        vuforiaCamera.initialize();
    }

    @Override
    public void loop() {

    }
}
