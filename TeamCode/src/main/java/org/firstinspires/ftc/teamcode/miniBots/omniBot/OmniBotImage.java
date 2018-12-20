package org.firstinspires.ftc.teamcode.miniBots.omniBot;

import android.app.Activity;
import android.content.Context;
import android.support.annotation.IdRes;
import android.view.SurfaceView;
import android.view.ViewGroup;
import android.widget.LinearLayout;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerNotifier;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeManagerImpl;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
//import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.drive.control.PID;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.JavaCameraView;
import org.opencv.android.OpenCVLoader;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

//@TeleOp(name = "OmniBotImage", group = "omnibot")
public class OmniBotImage {//extends OmniBotConfig implements CameraBridgeViewBase.CvCameraViewListener2, OpModeManagerNotifier.Notifications {
//
//    static String load = "Not loaded!";
//
//    ElapsedTime cVTimer;
//    public double elapseCV;
//
//
//    static {
//        if (!OpenCVLoader.initDebug()) {
//            load = "Error Loading!";
//        } else {
//            load = "Loaded Successfully!";
//        }
//    }
//
//    static double location = 1;
//    static double size = 1;
//    static double pos = 0;
//    public double FPS = 0;
//    public PID pid = new PID(0.0003,0.00000007,0.0007);
//
//
//    private ViewGroup cameraMonitorView;
//    private JavaCameraView cameraView;
//    AppUtil appUtil = AppUtil.getInstance();
//    Activity activity = appUtil.getActivity();
//    Context context = AppUtil.getDefContext();
//    OpModeManagerImpl opModeManager;
//    private com.qualcomm.robotcore.util.Range coeff;
//
//    @Override
//    public void init() {
//        config(this);
//        pid.setOutputLimits(-1,1);
//        pid.reset();
//        @IdRes int cameraMonitorViewId = context.getResources().getIdentifier("cameraMonitorViewId", "id", context.getPackageName());
//        appUtil.runOnUiThread(() -> {
//            cameraView = new JavaCameraView(activity,
//                            JavaCameraView.CAMERA_ID_BACK);
//            cameraView.setVisibility(cameraMonitorViewId == 0 ? SurfaceView.INVISIBLE : SurfaceView.VISIBLE);
//            cameraView.setCvCameraViewListener(this);
//            if (cameraMonitorViewId == 0) {
//                cameraMonitorView = (ViewGroup) activity.findViewById(android.R.id.content);
//            } else {
//                cameraMonitorView = (LinearLayout) activity.findViewById(cameraMonitorViewId);
//            }
//            cameraMonitorView.addView(cameraView);
//            cameraView.enableView();
//        });
//        opModeManager = OpModeManagerImpl.getOpModeManagerOfActivity(activity);
//        if (opModeManager != null) {
//            opModeManager.registerListener(this);
//        }
//
//        cVTimer = new ElapsedTime();
//    }
//
//    @Override
//    public void init_loop() {
//        telemetry.addData("OpenCV", load);
//        pid.reset();
//    }
//
//    @Override
//    public void loop() {
//
//        telemetry.addData("Size", size);
//        telemetry.addData("Location", location);
//        telemetry.addData("Position", pos);
//
//        if (Math.abs(pos)>100){
//
//            robotHandler.drive.tank.setPower(-pid.getOutput(pos,0),-pid.getOutput(pos,0));
//        }
//        else if(size<50000){
//            robotHandler.drive.tank.setPower(-0.2,0.2);
//            pid.reset();
//
//        }
//        else{
//            robotHandler.drive.tank.setPower(0,0);
//            pid.reset();
//        }
//        //3.6mm focal
//        //0.00112mm
////        robotHandler.drive.mecanum.setMecanumThridPerson();
//
//    }
//
//    public static List<MatOfPoint> getloc(Mat input, Telemetry telemetry) {
//        try {
//            Mat crop = input.clone();
//            Imgproc.cvtColor(crop,crop,Imgproc.COLOR_RGB2HSV);
//            //Create Mask
//            Mat mask = new Mat();
//            Core.inRange(crop, new Scalar(30, 40, 10), new Scalar(70, 255, 255), mask);
//            //Find Contours
//            List<MatOfPoint> contours = new ArrayList<MatOfPoint>();
//            Imgproc.findContours(mask, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
//            double maxArea = 0;
//            List<MatOfPoint> biggest = new ArrayList<>();
//            int index = -1;
//            Iterator<MatOfPoint> each = contours.iterator();
//            while (each.hasNext()) {
//                MatOfPoint wrapper = each.next();
//                double area = Imgproc.contourArea(wrapper);
//                if (area > maxArea) {
//                    maxArea = area;
//                    List<MatOfPoint> current = new ArrayList<>();
//                    current.add(wrapper);
//                    biggest = current;
//                }
//            }
//
//            //Centroid setup
//            Moments mmnts = Imgproc.moments(mask, true);
//            location = mmnts.get_m10() / mmnts.get_m00();
//            pos = (crop.width()/2)-location;
//            size = maxArea;
//
//            return biggest;
//            //}
//        } catch (Exception e)
//
//        {
//            return null;
//        }
//    }
//
//    @Override
//    public void onCameraViewStarted(int width, int height) {
//
//    }
//
//    @Override
//    public void onCameraViewStopped() {
//
//    }
//
//    @Override
//    public Mat onCameraFrame(CameraBridgeViewBase.CvCameraViewFrame inputFrame) {
//
//        FPS = Math.round(1/((cVTimer.nanoseconds() - elapseCV) / 1000000000)*100)/100;;
//        Mat frame = inputFrame.rgba();
//        Imgproc.drawContours(frame,getloc(frame,telemetry), -1, new Scalar(255,0 , 0), 5);
//        Imgproc.putText(frame,"FPS: "+FPS,new Point(375,100),Core.FONT_HERSHEY_PLAIN,2,new Scalar(0,255,0),4);
//        elapseCV = cVTimer.nanoseconds();
//
//        return frame;
//    }
//
//
//    @Override
//    public void onOpModePreInit(OpMode opMode) {
//
//    }
//
//    @Override
//    public void onOpModePreStart(OpMode opMode) {
//
//    }
//
//    @Override
//    public void onOpModePostStop(OpMode opMode) {
//        if (cameraView != null) {
//            appUtil.runOnUiThread(() -> {
//                cameraMonitorView.removeView(cameraView);
//                cameraView.disableView();
//                cameraView = null;
//            });
//
//        }
//        if (opModeManager != null) {
//            opModeManager.unregisterListener(this);
//        }
//    }
}
