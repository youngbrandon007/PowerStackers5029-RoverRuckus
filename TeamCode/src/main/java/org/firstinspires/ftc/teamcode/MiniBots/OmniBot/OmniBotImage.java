package org.firstinspires.ftc.teamcode.MiniBots.OmniBot;

import android.graphics.Bitmap;

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

    static{
        if(!OpenCVLoader.initDebug()){

        }else{

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
    public void loop() {
        try {
            getloc(PineappleRelicRecoveryVuforia.getImageFromFrame(vuforia.getFrameQueue().take(), PIXEL_FORMAT.RGB565), listener, vuforia.getCameraCalibration(), telemetry);
        }catch(Exception e){
            telemetry.addData("Error", e.getMessage());
        }
    }

    public static double getloc(Image img, VuforiaTrackableDefaultListener track, CameraCalibration camCal, Telemetry telemetry) {
        try {
            //OpenGLMatrix pose = track.getRawPose();
            //if (pose != null && img != null && img.getPixels() != null) {
                //Matrix34F rawPose = new Matrix34F();
                //float[] poseData = Arrays.copyOfRange(pose.transposed().getData(), 0, 12);
                //rawPose.setData(poseData);
                //float[][] corners = new float[4][2];
                //corners[0] = Tool.projectPoint(camCal, rawPose, new Vec3F(120, -55, 100)).getData();//UL TODO FIND NEW LOCATIONS
                //corners[1] = Tool.projectPoint(camCal, rawPose, new Vec3F(340, -55, 100)).getData();//UR TODO FIND NEW LOCATIONS
                //corners[2] = Tool.projectPoint(camCal, rawPose, new Vec3F(340, -300, 100)).getData();//LR TODO FIND NEW LOCATIONS
                //corners[3] = Tool.projectPoint(camCal, rawPose, new Vec3F(120, -300, 100)).getData();//LL TODO FIND NEW LOCATIONS
                Bitmap bm = Bitmap.createBitmap(img.getWidth(), img.getHeight(), Bitmap.Config.RGB_565);
                ByteBuffer pix = img.getPixels();
                bm.copyPixelsFromBuffer(pix);
                telemetry.addData("size" , "" + img.getWidth() + img.getHeight());
                telemetry.update();
                SaveImage(bm, "original");
                Mat crop = new Mat(bm.getHeight(), bm.getWidth(), CvType.CV_8UC3); //C3
                Utils.bitmapToMat(bm, crop);
//                float x = Math.min(Math.min(corners[1][0], corners[3][0]), Math.min(corners[0][0], corners[2][0]));
//                float y = Math.min(Math.min(corners[1][1], corners[3][1]), Math.min(corners[0][1], corners[2][1]));
//                float width = Math.max(Math.abs(corners[0][0] - corners[2][0]), Math.abs(corners[1][0] - corners[3][0]));
//                float height = Math.max(Math.abs(corners[0][1] - corners[2][1]), Math.abs(corners[1][1] - corners[3][1]));
//                x = Math.max(x, 0);
//                y = Math.max(y, 0);
//                if (width < 20 || height < 20) {
//                    return NON_NON;
//                }
                //width = (x + width > crop.cols()) ? crop.cols() - x : width;
                //height = (x + height > crop.rows()) ? crop.rows() - x : height;
                //Mat cropped = new Mat(crop, new Rect((int) x, (int) y, (int) width, (int) height));
                //SaveImage(matToBitmap(cropped), "crop");
                Imgproc.cvtColor(crop, crop, Imgproc.COLOR_RGB2HSV_FULL);
                Mat mask = new Mat();
                Core.inRange(crop, new Scalar(50, 20, 70), new Scalar(255, 255, 120), mask);
                SaveImage(matToBitmap(mask), "mask");
                Moments mmnts = Imgproc.moments(mask, true);
                telemetry.addData("Data", mmnts.get_m10() / mmnts.get_m00());
                return mmnts.get_m10() / mmnts.get_m00();

            //}
        } catch (Exception e)

        {
            return -1;
        }
    }

}
