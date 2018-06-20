package org.firstinspires.ftc.teamcode.MiniBots.OmniBot;

import android.graphics.Bitmap;
import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.vuforia.CameraCalibration;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.Vuforia.PineappleRelicRecoveryVuforia;
import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.OpenCVLoader;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.nio.ByteBuffer;

import static org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.Vuforia.PineappleRelicRecoveryVuforia.SaveImage;
import static org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.Vuforia.PineappleRelicRecoveryVuforia.matToBitmap;

@TeleOp(name = "OmniBotImage")
public class OmniBotImage extends  OmniBotConfig{

    VuforiaLocalizer vuforia;
    VuforiaTrackables relicTrackables;
    VuforiaTrackable relicTemplate;
    VuforiaTrackableDefaultListener listener;


    static String load = "Not loaded!";

    static{
        if(!OpenCVLoader.initDebug()){
            load = "Error Loading!";
        }else{
            load = "Loaded Successfully!";
        }
    }

    @Override
    public void init() {
        config(this);


        //Camera init
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AdB8VB7/////AAAAGcfBp9I80URFkfBQFUyM+ptmQXBAMGx0svJKz7QE2nm20mBc/zI5sZNHfuP/ziIm+sYnO7fvPqUbFs8QWjRyXVEDmW4mMj+S+l+yaYRkpGZ/pmHyXiDb4aemHx0m70BulMNIce4+NVaCW5S/5BWNNev/AU0P+uWnHYuKNWzD2dPaRuprC4R6b/DgD1zeio1xlssYb9in9mfzn76gChOrE5B0ql6Q9FiHC5cTdacq2lKjm5nlkTiXz1e2jhVK3SddGoqM4FQ3mFks7/A88hFzlPfIIk45K2Lh7GvcVjuIiqNj5mTLaZJVqlsKdTQnKS4trJcc1YV9sjdbmh1agtn1UePy91fDj9uWSBdXvpIowv4B";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        vuforia.setFrameQueueCapacity(1);
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
        relicTrackables = vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary
        listener = (VuforiaTrackableDefaultListener) relicTemplate.getListener();
        relicTrackables.activate();
    }

    @Override
    public void init_loop(){
        telemetry.addData("OpenCV", load);
    }

    @Override
    public void loop() {
        try {
            getloc(PineappleRelicRecoveryVuforia.getImageFromFrame(vuforia.getFrameQueue().take(), PIXEL_FORMAT.RGB565),  telemetry);
        }catch(Exception e){
            telemetry.addData("Error", e.getMessage());
        }
    }

    public static double getloc(Image img,  Telemetry telemetry) {
        try {
            Bitmap bm = Bitmap.createBitmap(img.getWidth(), img.getHeight(), Bitmap.Config.RGB_565);
            ByteBuffer pix = img.getPixels();
            bm.copyPixelsFromBuffer(pix);
            telemetry.addData("size" , "" + img.getWidth() +"," + img.getHeight());
            SaveImage(bm, "original");
            Mat crop = new Mat(bm.getHeight(), bm.getWidth(), CvType.CV_8UC3); //C3
            Utils.bitmapToMat(bm, crop);
            Scalar min = rgbToScalar(200,50,0);
            Scalar max = rgbToScalar(255,150,100);
            Imgproc.cvtColor(crop, crop, Imgproc.COLOR_RGB2HSV_FULL);
            Mat mask = new Mat();
            //new Scalar(50, 20, 70), new Scalar(255, 255, 120)
            Core.inRange(crop, min, max, mask);
            SaveImage(matToBitmap(mask), "mask");
            Moments mmnts = Imgproc.moments(mask, true);
            telemetry.addData("Data", mmnts.get_m10() / mmnts.get_m00());
            int[] color = getARGB(bm.getPixel(0,0));
            telemetry.addData("r",color[0]);
            telemetry.addData("g",color[1]);
            telemetry.addData("b",color[2]);
            return mmnts.get_m10() / mmnts.get_m00();

            //}
        } catch (Exception e)

        {
            return -1;
        }
    }


    public static int[] getARGB(int p){
        int R = (p & 0xff0000) >> 16;
        int G = (p & 0xff00) >> 8;
        int B = p & 0xff;
        return new int[]{R,G,B};
    }

    public static Scalar rgbToScalar(int r, int g, int b){
        float[] hsv = new float[3];
        Color.RGBToHSV(r, g, b, hsv);
        return new Scalar(hsv[0], hsv[1], hsv[2]);
    }
}
