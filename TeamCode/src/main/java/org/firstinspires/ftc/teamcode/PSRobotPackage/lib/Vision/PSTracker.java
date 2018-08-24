package org.firstinspires.ftc.teamcode.PSRobotPackage.lib.Vision;

import org.opencv.core.Mat;

import java.util.LinkedHashMap;

public abstract class PSTracker {
    private boolean enabled = true;
    private LinkedHashMap<String, Mat> intermediates = new LinkedHashMap<>();

    void internalProcessFrame(Mat frame, double timestamp) {
        if (enabled) {
            intermediates.clear();
            processFrame(frame, timestamp);
        }
    }

    public void enable() {
        enabled = true;
    }

    public void disable() {
        enabled = false;
    }

    public boolean isEnabled() {
        return enabled;
    }

    public LinkedHashMap<String, Mat> getIntermediates() {
        return intermediates;
    }

    protected void addIntermediate(String key, Mat value) {
        intermediates.put(key, value);
    }

    public abstract void init(PSCamera camera);
    public abstract void processFrame(Mat frame, double timestamp);
    public abstract void drawOverlay(PSOverlay overlay, int imageWidth, int imageHeight, boolean debug);
}
