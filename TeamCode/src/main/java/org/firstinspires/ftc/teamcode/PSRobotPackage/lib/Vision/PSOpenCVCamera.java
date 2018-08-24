package org.firstinspires.ftc.teamcode.PSRobotPackage.lib.Vision;

import android.view.SurfaceView;
import android.view.ViewGroup;
import android.widget.LinearLayout;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.PSRobotPackage.lib.PSEnum;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.JavaCameraView;
import org.opencv.core.Mat;

public class PSOpenCVCamera extends PSCamera implements CameraBridgeViewBase.CvCameraViewListener2 {
    private ViewGroup cameraMonitorView;
    private JavaCameraView cameraView;

    public PSOpenCVCamera(PSEnum.CameraDirection cameraDirection) {
        super(cameraDirection);
    }

    @Override
    protected void doInitialize() {
        appUtil.runOnUiThread(() -> {
            cameraView = new JavaCameraView(activity, cameraDirection == PSEnum.CameraDirection.FRONT ? JavaCameraView.CAMERA_ID_FRONT : JavaCameraView.CAMERA_ID_BACK);
            cameraView.setVisibility(cameraMonitorViewId == 0 ? SurfaceView.INVISIBLE : SurfaceView.VISIBLE);
            cameraView.setCvCameraViewListener(this);
            if (cameraMonitorViewId == 0) {
                cameraMonitorView = (ViewGroup) activity.findViewById(android.R.id.content);
            } else {
                cameraMonitorView = (LinearLayout) activity.findViewById(cameraMonitorViewId);
            }
            cameraMonitorView.addView(cameraView);
            cameraView.enableView();
        });
    }

    @Override
    public void close() {
        if (cameraView != null) {
            appUtil.runOnUiThread(() -> {
                cameraMonitorView.removeView(cameraView);
                cameraView.disableView();
                cameraView = null;
            });
        }
    }

    @Override
    public Properties getProperties() {
        return new OpenCVProperties(cameraView);
    }

    @Override
    public void onCameraViewStarted(int width, int height) {

    }

    @Override
    public void onCameraViewStopped() {

    }

    @Override
    public Mat onCameraFrame(CameraBridgeViewBase.CvCameraViewFrame inputFrame) {
        Mat frame = inputFrame.rgba();
        onFrame(frame, System.nanoTime()/ Math.pow(10, 9));
        return frame;
    }

    @Override
    protected void onFrame(Mat frame, double timestamp) {
        super.onFrame(frame, timestamp);
        PSOverlay overlay = new PSOverlay(frame);
        synchronized (trackers) {
            for (PSTracker tracker : trackers) {
                if (tracker.isEnabled()) {

                    overlay.setScalingFactor(1);

                    tracker.drawOverlay(overlay, frame.cols(), frame.rows(), true);
                }
            }
        }
    }

    public class OpenCVProperties implements Properties {
        private JavaCameraView cameraView;

        public OpenCVProperties(JavaCameraView cameraView) {
            this.cameraView = cameraView;
        }

        @Override
        public double getHorizontalFocalLengthPx(double imageWidth) {
            double fov = Math.toRadians(cameraView.getCamera().getParameters().getHorizontalViewAngle());
            return (imageWidth * 0.5) / Math.tan(0.5 * fov);
        }
    }
}
