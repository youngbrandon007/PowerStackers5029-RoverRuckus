package org.firstinspires.ftc.teamcode.PSRobotPackage.lib.Vision;

import android.app.Activity;
import android.content.Context;
import android.support.annotation.IdRes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerNotifier;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeManagerImpl;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.PSRobotPackage.lib.PSEnum;
import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.core.Mat;

import java.util.concurrent.CountDownLatch;

public abstract class PSCamera implements OpModeManagerNotifier.Notifications {

    protected AppUtil appUtil = AppUtil.getInstance();
    protected Activity activity;
    protected OpModeManagerImpl opModeManager;

    Context context = AppUtil.getDefContext();
    @IdRes
    public int cameraMonitorViewId;
    private boolean initialized;
    protected PSEnum.CameraDirection cameraDirection  = PSEnum.CameraDirection.BACK;;

    public PSCamera(PSEnum.CameraDirection cameraDirection) {
        this.activity = appUtil.getActivity();
        this.cameraDirection = cameraDirection;
        cameraMonitorViewId = context.getResources().getIdentifier("cameraMonitorViewId", "id", context.getPackageName());
        opModeManager = OpModeManagerImpl.getOpModeManagerOfActivity(activity);
        if (opModeManager != null) {
            opModeManager.registerListener(this);
        }
    }

    public void initialize() {
        if (!initialized) {
            final CountDownLatch openCvInitialized = new CountDownLatch(1);

            final BaseLoaderCallback loaderCallback = new BaseLoaderCallback(activity) {
                @Override
                public void onManagerConnected(int status) {
                    switch (status) {
                        case LoaderCallbackInterface.SUCCESS: {
//                            Log.i(TAG, "OpenCV loaded successfully");
                            openCvInitialized.countDown();
                            break;
                        }
                        default: {
                            super.onManagerConnected(status);
                            break;
                        }
                    }
                }
            };

            appUtil.runOnUiThread(() -> OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION, activity, loaderCallback));

            try {
                openCvInitialized.await();
            } catch (InterruptedException e) {
//                Log.w(TAG, e);
            }

            doInitialize();
        }
    }

    protected void onFrame(Mat frame, double timestamp) {
//        synchronized (trackers) {
//            for (Tracker tracker : trackers) {
//                tracker.internalProcessFrame(frame, timestamp);
//            }
//        }
    }

    @Override
    public void onOpModePreInit(OpMode opMode) {

    }

    @Override
    public void onOpModePreStart(OpMode opMode) {

    }

    @Override
    public void onOpModePostStop(OpMode opMode) {
        close();
        if (opModeManager != null) {
            opModeManager.unregisterListener(this);
        }
    }

    protected abstract void doInitialize();
    public abstract void close();
    public abstract Properties getProperties();

    public interface Properties {
        /** @return camera's horizontal (along x-axis) focal length in pixels */
        double getHorizontalFocalLengthPx(double imageWidth);
    }
}
