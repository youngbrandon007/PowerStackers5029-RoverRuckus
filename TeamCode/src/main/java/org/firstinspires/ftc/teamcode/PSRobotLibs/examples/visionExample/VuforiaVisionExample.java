package org.firstinspires.ftc.teamcode.PSRobotLibs.examples.visionExample;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.PSEnum;
import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.vision.PSVuforiaCamera;

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
