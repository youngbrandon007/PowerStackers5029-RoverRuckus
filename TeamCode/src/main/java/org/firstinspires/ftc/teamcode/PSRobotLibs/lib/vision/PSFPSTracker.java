package org.firstinspires.ftc.teamcode.PSRobotLibs.lib.vision;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;

import java.util.Locale;

public class PSFPSTracker extends PSTracker {
    private double lastTimestamp, timeChange;

    @Override
    public void init(PSCamera camera) {

    }

    @Override
    public void processFrame(Mat frame, double timestamp) {
        if (lastTimestamp!=0){
            timeChange = timestamp - lastTimestamp;
        }
        lastTimestamp = timestamp;
    }

    @Override
    public void drawOverlay(PSOverlay overlay, int imageWidth, int imageHeight, boolean debug) {
        overlay.putText(String.format(Locale.ENGLISH, "%.2f FPS", 1 / timeChange), PSOverlay.TextAlign.RIGHT, new Point(imageWidth - 5, 45), new Scalar(0, 0, 255), 45);

    }

    public double getFPS(){
        return timeChange;
    }
}
