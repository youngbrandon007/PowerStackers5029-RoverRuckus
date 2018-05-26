package org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.Vuforia;

import android.graphics.Bitmap;
import android.support.annotation.Nullable;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.vuforia.CameraCalibration;
import com.vuforia.Image;
import com.vuforia.Matrix34F;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Tool;
import com.vuforia.Vec3F;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PineappleEnum;
import org.firstinspires.ftc.teamcode.R;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.util.Arrays;

import static org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PineappleEnum.JewelState.BLUE_RED;
import static org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PineappleEnum.JewelState.RED_BLUE;


/**
 * Created by ftcpi on 9/16/2017.
 */
@Autonomous(name = "VuforiaTestJewel", group = "Linear Opmode")
@Disabled
public class VuforiaTest extends LinearOpMode {
//    public static float[] blueLowHSV = {240,90,34};
//    public static  float[] blueHighHSV= {186,33,95};
//    public final static Scalar blueLow = new Scalar(stringArrtoIntArr(hsvToRgb(blueLowHSV))[0], stringArrtoIntArr(hsvToRgb(blueLowHSV))[1] , stringArrtoIntArr(hsvToRgb(blueLowHSV))[2]);
//    public final static Scalar blueHigh = new Scalar(stringArrtoIntArr(hsvToRgb(blueHighHSV))[0], stringArrtoIntArr(hsvToRgb(blueHighHSV))[1] , stringArrtoIntArr(hsvToRgb(blueHighHSV))[2]);
//    public final static Scalar blueLow = new Scalar(108, 0 , 220);
//    public final static Scalar blueHigh = new Scalar(178, 255 , 255);
    @Override
    public void runOpMode() throws InterruptedException {
        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        params.vuforiaLicenseKey = "AdB8VB7/////AAAAGcfBp9I80URFkfBQFUyM+ptmQXBAMGx0svJKz7QE2nm20mBc/zI5sZNHfuP/ziIm+sYnO7fvPqUbFs8QWjRyXVEDmW4mMj+S+l+yaYRkpGZ/pmHyXiDb4aemHx0m70BulMNIce4+NVaCW5S/5BWNNev/AU0P+uWnHYuKNWzD2dPaRuprC4R6b/DgD1zeio1xlssYb9in9mfzn76gChOrE5B0ql6Q9FiHC5cTdacq2lKjm5nlkTiXz1e2jhVK3SddGoqM4FQ3mFks7/A88hFzlPfIIk45K2Lh7GvcVjuIiqNj5mTLaZJVqlsKdTQnKS4trJcc1YV9sjdbmh1agtn1UePy91fDj9uWSBdXvpIowv4B";
        params.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;


        VuforiaLocalizer locale = ClassFactory.createVuforiaLocalizer(params);
        locale.setFrameQueueCapacity(1);
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
        VuforiaTrackables relicTrackables;
        relicTrackables = locale.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackableDefaultListener track = (VuforiaTrackableDefaultListener) relicTrackables.get(0).getListener();
        relicTrackables.activate();
        waitForStart();
        PineappleEnum.JewelState state;
        PineappleEnum.JewelState lastState;
        boolean hasJewelConfig = false;
        while (opModeIsActive()&& !hasJewelConfig){
            state = PineappleRelicRecoveryVuforia.getJewelConfig(PineappleRelicRecoveryVuforia.getImageFromFrame(locale.getFrameQueue().take(), PIXEL_FORMAT.RGB565),track, locale.getCameraCalibration(), telemetry);
            if (state == PineappleEnum.JewelState.NON_NON) {
                telemetry.addData("Config ", "NON");
            } else if (state == PineappleEnum.JewelState.BLUE_RED) {
                telemetry.addData("Config ", "BLUE RED");
            } else if (state == PineappleEnum.JewelState.RED_BLUE) {
                telemetry.addData("Config ", "RED BLUE");
            }
            telemetry.update();
        }


    }


}
