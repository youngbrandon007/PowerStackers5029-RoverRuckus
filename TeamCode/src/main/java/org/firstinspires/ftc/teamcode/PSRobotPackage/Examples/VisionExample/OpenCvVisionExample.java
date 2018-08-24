package org.firstinspires.ftc.teamcode.PSRobotPackage.Examples.VisionExample;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.PSRobotPackage.lib.PSEnum;
import org.firstinspires.ftc.teamcode.PSRobotPackage.lib.Vision.PSFPSTracker;
import org.firstinspires.ftc.teamcode.PSRobotPackage.lib.Vision.PSOpenCVCamera;

public class OpenCvVisionExample extends OpMode {
    private PSOpenCVCamera camera;
    private PSFPSTracker tracker;

    @Override
    public void init() {
        camera = new PSOpenCVCamera(PSEnum.CameraDirection.BACK);
        tracker = new PSFPSTracker();
        camera.addTracker(tracker);
        camera.initialize();

    }

    @Override
    public void loop() {
        telemetry.addData("FPS:", tracker.getFPS());
        telemetry.update();
    }
}
