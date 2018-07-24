package org.firstinspires.ftc.teamcode.MiniBots.OmniBot;

import android.app.Activity;
import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.Color;
import android.support.annotation.IdRes;
import android.view.SurfaceView;
import android.view.ViewGroup;
import android.widget.LinearLayout;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerNotifier;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeManagerImpl;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PSConfigOpMode;
import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.Vision.PineappleVision;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.JavaCameraView;
import org.opencv.android.OpenCVLoader;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;


import static org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.Vision.PineappleVision.SaveImage;
import static org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.Vision.PineappleVision.matToBitmap;

@TeleOp(name = "OmniBotImage", group = "omnibot")
public class OmniBotImage extends PSConfigOpMode implements CameraBridgeViewBase.CvCameraViewListener2, OpModeManagerNotifier.Notifications {

//    VuforiaLocalizer vuforia;
//    VuforiaTrackables relicTrackables;
//    VuforiaTrackable relicTemplate;
//    VuforiaTrackableDefaultListener listener;


    static String load = "Not loaded!";

    ElapsedTime fpsTimer;
    long elapse;


    static {
        if (!OpenCVLoader.initDebug()) {
            load = "Error Loading!";
        } else {
            load = "Loaded Successfully!";
        }
    }

    static double location = 1;
    static double size = 1;
    static double pos = 0;
    private ViewGroup cameraMonitorView;
    private JavaCameraView cameraView;
    AppUtil appUtil = AppUtil.getInstance();
    Activity activity = appUtil.getActivity();
    Context context = AppUtil.getDefContext();
    OpModeManagerImpl opModeManager;

    @Override
    public void init() {
//        config(this);


        //Camera init
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
//        parameters.vuforiaLicenseKey = "AdB8VB7/////AAAAGcfBp9I80URFkfBQFUyM+ptmQXBAMGx0svJKz7QE2nm20mBc/zI5sZNHfuP/ziIm+sYnO7fvPqUbFs8QWjRyXVEDmW4mMj+S+l+yaYRkpGZ/pmHyXiDb4aemHx0m70BulMNIce4+NVaCW5S/5BWNNev/AU0P+uWnHYuKNWzD2dPaRuprC4R6b/DgD1zeio1xlssYb9in9mfzn76gChOrE5B0ql6Q9FiHC5cTdacq2lKjm5nlkTiXz1e2jhVK3SddGoqM4FQ3mFks7/A88hFzlPfIIk45K2Lh7GvcVjuIiqNj5mTLaZJVqlsKdTQnKS4trJcc1YV9sjdbmh1agtn1UePy91fDj9uWSBdXvpIowv4B";
//ADD CAMERA DIRECTIOn
//        vuforia = ClassFactory.createVuforiaLocalizer(parameters);
//        vuforia.setFrameQueueCapacity(1);
//        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
//        relicTrackables = vuforia.loadTrackablesFromAsset("RelicVuMark");
        //relicTemplate = relicTrackables.get(0);
        //relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary
        //listener = (VuforiaTrackableDefaultListener) relicTemplate.getListener();
        //relicTrackables.activate();


        @IdRes int cameraMonitorViewId = context.getResources().getIdentifier("cameraMonitorViewId", "id", context.getPackageName());
        appUtil.runOnUiThread(() -> {
            cameraView = new JavaCameraView(activity,
                            JavaCameraView.CAMERA_ID_BACK);
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
        opModeManager = OpModeManagerImpl.getOpModeManagerOfActivity(activity);
        if (opModeManager != null) {
            opModeManager.registerListener(this);
        }

        fpsTimer = new ElapsedTime();
    }

    @Override
    public void init_loop() {
        telemetry.addData("OpenCV", load);
    }

    @Override
    public void loop() {

        double nanochange = fpsTimer.nanoseconds() - elapse;
        double millChange = nanochange / 1000000;

        elapse = fpsTimer.nanoseconds();
        telemetry.addData("Mill FT", millChange);
        telemetry.addData("FPS", 1/(millChange/1000));
        telemetry.addData("Size", size);
        telemetry.addData("Location", location);

//        try {
//            getloc(PineappleVision.getImageFromFrame(vuforia.getFrameQueue().take(), PIXEL_FORMAT.RGB565), telemetry);
//        } catch (Exception e) {
//            telemetry.addData("Error", e.getMessage());
//        }
//        if (Math.abs(pos)>50){
//            robotHandler.drive.tank.setPower(pos*0.001,pos*0.001);
//        }
////        else if(size>30000){
////            robotHandler.drive.tank.setPower(-0.2,0.2);
////
////        }
//        else{
//            robotHandler.drive.tank.setPower(0,0);
//        }

    }

    public static List<MatOfPoint> getloc(Mat input, Telemetry telemetry) {
        try {
            Mat crop = input.clone();
//            SaveImage(matToBitmap(crop), "INPUT");
            Imgproc.cvtColor(crop,crop,Imgproc.COLOR_RGB2HSV);
//            SaveImage(matToBitmap(crop), "original");
//            Scalar min = new Scalar(70, 150, 50);
//            Scalar max = new Scalar(100, 255, 255);
            Scalar min = new Scalar(40, 150, 50);
            Scalar max = new Scalar(70, 255, 255);
            //convert color format
            //Create Mask
            Mat mask = new Mat();
            Core.inRange(crop, min, max, mask);
//            SaveImage(matToBitmap(mask), "mask");

            //Find Contours
            List<MatOfPoint> contours = new ArrayList<MatOfPoint>();
            Imgproc.findContours(mask, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
            Imgproc.drawContours(mask, contours, -1, new Scalar(0,255,255), 3);
//            SaveImage(matToBitmap(mask), "Contour");
            // Find max contour area
            double maxArea = 0;
            List<MatOfPoint> biggest = new ArrayList<>();
            int index = -1;
            Iterator<MatOfPoint> each = contours.iterator();
            while (each.hasNext()) {
                MatOfPoint wrapper = each.next();
                double area = Imgproc.contourArea(wrapper);
                if (area > maxArea) {
                    maxArea = area;
                    List<MatOfPoint> current = new ArrayList<>();
                    current.add(wrapper);
                    biggest = current;
                }
            }
            Imgproc.cvtColor(crop, crop, Imgproc.COLOR_HSV2BGR);

            //Centroid setup
            Moments mmnts = Imgproc.moments(mask, true);


            location = mmnts.get_m10() / mmnts.get_m00();
            pos = (crop.width()/2)-location;
            size = maxArea;

            return biggest;
            //}
        } catch (Exception e)

        {
            return null;
        }
    }

    public static int[] getARGB(int p) {
        int R = (p & 0xff0000) >> 16;
        int G = (p & 0xff00) >> 8;
        int B = p & 0xff;
        return new int[]{R, G, B};
    }

    public static Scalar rgbToScalar(int r, int g, int b) {
        float[] hsv = new float[3];
        Color.RGBToHSV(r, g, b, hsv);
        return new Scalar(hsv[2], hsv[1], hsv[0]);
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


        Imgproc.drawContours(frame,getloc(frame,telemetry), -1, new Scalar(255,0 , 0), 5);

        return frame;
    }


    @Override
    public void config(OpMode opMode) {

    }

    @Override
    public void onOpModePreInit(OpMode opMode) {

    }

    @Override
    public void onOpModePreStart(OpMode opMode) {

    }

    @Override
    public void onOpModePostStop(OpMode opMode) {
        if (cameraView != null) {
            appUtil.runOnUiThread(() -> {
                cameraMonitorView.removeView(cameraView);
                cameraView.disableView();
                cameraView = null;
            });

        }
        if (opModeManager != null) {
            opModeManager.unregisterListener(this);
        }
    }
}
