package org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.Vuforia;

import android.graphics.Bitmap;
import android.os.Environment;
import android.support.annotation.Nullable;

import com.vuforia.CameraCalibration;
import com.vuforia.Image;
import com.vuforia.Matrix34F;
import com.vuforia.Tool;
import com.vuforia.Vec3F;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PineappleEnum;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.io.File;
import java.io.FileOutputStream;
import java.nio.ByteBuffer;
import java.util.Arrays;

import static org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PineappleEnum.JewelState.BLUE_RED;
import static org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PineappleEnum.JewelState.NON_NON;
import static org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PineappleEnum.JewelState.RED_BLUE;


/**
 * Created by young on 9/13/2017.
 */

public class PineappleRelicRecoveryVuforia extends PineappleVuforia {
    //    public final static Scalar blueLow = new Scalar(108, 0 , 220);
//    public final static Scalar blueLow = new Scalar(220, 99, 45);
//
//    public final static Scalar blueHigh = new Scalar(192, 84, 100);
    //    public final static Scalar blueHigh = new Scalar(178, 255 , 255);
    VuforiaTrackables relicTrackables;
    VuforiaTrackable relicTemplate;

    public PineappleRelicRecoveryVuforia(int maxItemCount, VuforiaLocalizer.CameraDirection direction, VuforiaLocalizer.Parameters.CameraMonitorFeedback feedback, String vuforiaLicenseKey) {
        super(maxItemCount, direction, feedback, vuforiaLicenseKey);

    }

    public void addRelicRecoveryTrackables() {
        relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); //
    }

    public void startRelicRecoveryTracking() {
        relicTrackables.activate();
    }

    public PineappleEnum.VuMarkLocation getTrackingRelic() {
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);

        PineappleEnum.VuMarkLocation loc = PineappleEnum.VuMarkLocation.UNKNOWN;

        if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
                /* Found an instance of the template. In the actual game, you will probably
                 * loop until this Condition occurs, then move on to act accordingly depending
                 * on which VuMark was visible. */
            resources.telemetry.addData("VuMark", "%s visible", vuMark);

                /* For fun, we also exhibit the navigational pose. In the Relic Recovery game,
                 * it is perhaps unlikely that you will actually need to act on this pose information, but
                 * we illustrate it nevertheless, for completeness. */
            OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) relicTemplate.getListener()).getPose();
            String strPose = format(pose);
            resources.feedBack.sayFeedBackWithOutUpdate("Pose", strPose);

            if (strPose.toLowerCase().equals("center")) {
                loc = PineappleEnum.VuMarkLocation.CENTER;
            }
            if (strPose.toLowerCase().equals("left")) {
                loc = PineappleEnum.VuMarkLocation.LEFT;
            }
            if (strPose.toLowerCase().equals("right")) {
                loc = PineappleEnum.VuMarkLocation.RIGHT;
            }
                /* We further illustrate how to decompose the pose into useful rotational and
                 * translational components */
            if (pose != null) {
                VectorF trans = pose.getTranslation();
                Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                // Extract the X, Y, and Z components of the offset of the target relative to the robot
                double tX = trans.get(0);
                double tY = trans.get(1);
                double tZ = trans.get(2);

                // Extract the rotational components of the target relative to the robot
                double rX = rot.firstAngle;
                double rY = rot.secondAngle;
                double rZ = rot.thirdAngle;
            }
        } else {
            resources.feedBack.sayFeedBackWithOutUpdate("VuMark", "not visible");
        }

        resources.feedBack.update();
        return loc;
    }

    //    public static PineappleEnum.JewelState getJewelState(Image img,VuforiaTrackableDefaultListener track, CameraCalibration cameraCalibration) throws InterruptedException {
//        return getJewelConfig(getImageFromFrame(vuforia.getFrameQueue().take(), PIXEL_FORMAT.RGB565),track, vuforia.getCameraCalibration());
//    }
    public static void SaveImage(Bitmap finalBitmap, String name) {

        String root = Environment.getExternalStorageDirectory().getAbsolutePath();
        File myDir = new File(root + "/saved_images");
        myDir.mkdirs();

        String fname = name + ".jpg";
        File file = new File(myDir, fname);
        if (file.exists()) file.delete();
        try {
            FileOutputStream out = new FileOutputStream(file);
            finalBitmap.compress(Bitmap.CompressFormat.JPEG, 90, out);
            out.flush();
            out.close();

        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    public static PineappleEnum.JewelState getJewelConfig(Image img, VuforiaTrackableDefaultListener track, CameraCalibration camCal, Telemetry telemetry) {
        try {
            OpenGLMatrix pose = track.getRawPose();
            if (pose != null && img != null && img.getPixels() != null) {
                Matrix34F rawPose = new Matrix34F();
                float[] poseData = Arrays.copyOfRange(pose.transposed().getData(), 0, 12);
                rawPose.setData(poseData);
                float[][] corners = new float[4][2];
                corners[0] = Tool.projectPoint(camCal, rawPose, new Vec3F(120, -55, 50)).getData();//UL
                corners[1] = Tool.projectPoint(camCal, rawPose, new Vec3F(340, -55, 50)).getData();//UR
                corners[2] = Tool.projectPoint(camCal, rawPose, new Vec3F(340, -300, 50)).getData();//LR
                corners[3] = Tool.projectPoint(camCal, rawPose, new Vec3F(120, -300, 50)).getData();//LL
                Bitmap bm = Bitmap.createBitmap(img.getWidth(), img.getHeight(), Bitmap.Config.RGB_565);
                ByteBuffer pix = img.getPixels();
                bm.copyPixelsFromBuffer(pix);
                SaveImage(bm, "orginialImage ");
                Mat crop = new Mat(bm.getHeight(), bm.getWidth(), CvType.CV_8UC3);
                Utils.bitmapToMat(bm, crop);
                float x = Math.min(Math.min(corners[1][0], corners[3][0]), Math.min(corners[0][0], corners[2][0]));
                float y = Math.min(Math.min(corners[1][1], corners[3][1]), Math.min(corners[0][1], corners[2][1]));
                float width = Math.max(Math.abs(corners[0][0] - corners[2][0]), Math.abs(corners[1][0] - corners[3][0]));
                float height = Math.max(Math.abs(corners[0][1] - corners[2][1]), Math.abs(corners[1][1] - corners[3][1]));
                x = Math.max(x, 0);
                y = Math.max(y, 0);
                if (width<20||height<20) {
                    return NON_NON;
                }
                width = (x + width > crop.cols()) ? crop.cols() - x : width;
                height = (x + height > crop.rows()) ? crop.rows() - x : height;
                Mat cropped = new Mat(crop, new Rect((int) x, (int) y, (int) width, (int) height));
                SaveImage(matToBitmap(cropped), "crop");
                Imgproc.cvtColor(cropped, cropped, Imgproc.COLOR_RGB2HSV_FULL);
                Mat mask = new Mat();
                Core.inRange(cropped, new Scalar(50, 20, 70), new Scalar(255, 255, 120), mask);
                SaveImage(matToBitmap(mask), "mask");
                Moments mmnts = Imgproc.moments(mask, true);
                telemetry.addData("Centroid X", (mmnts.get_m10() / mmnts.get_m00()));
                telemetry.addData("Centroid Y", (mmnts.get_m01() / mmnts.get_m00()));


                if ((mmnts.get_m10() / mmnts.get_m00()) < cropped.cols() / 2) {
                    return BLUE_RED;
                } else {
                    return RED_BLUE;
                }

            }
            return NON_NON;
        } catch (Exception e)

        {
            return NON_NON;
        }
    }

    public static void isPictureInFrame(Image img, VuforiaTrackableDefaultListener track, CameraCalibration camCal, Telemetry telemetry) {

            OpenGLMatrix pose = track.getRawPose();
            if (pose != null) {
                if (pose != null && img != null && img.getPixels() != null) {
                    Matrix34F rawPose = new Matrix34F();
                    float[] poseData = Arrays.copyOfRange(pose.transposed().getData(), 0, 12);
                    rawPose.setData(poseData);
                    float[] picture;
                    picture = Tool.projectPoint(camCal, rawPose, new Vec3F(0, 0, 0)).getData();
                    if (100<picture[0]&&picture[0]<(img.getWidth()-100)) {
                        telemetry.addLine("DETECTS INSIDE WIDTH");
                    }
                    if (100<picture[1]&&picture[1]<(img.getHeight()-100)) {
                        telemetry.addLine("DETECTS INSIDE Heigth");
                    }
                    telemetry.addData("PointX", picture[0]);
                    telemetry.addData("PointY", picture[1]);
                    telemetry.update();


                }

            }
        }


    public static Mat bitmapToMat(Bitmap bit, int cvType) {
        Mat newMat = new Mat(bit.getHeight(), bit.getWidth(), cvType);

        Utils.bitmapToMat(bit, newMat);

        return newMat;
    }

    public static Bitmap matToBitmap(Mat mat) {
        Bitmap newBit = Bitmap.createBitmap(mat.cols(), mat.rows(), Bitmap.Config.ARGB_8888);

        Utils.matToBitmap(mat, newBit);

        return newBit;
    }

    @Nullable
    public static Image getImageFromFrame(VuforiaLocalizer.CloseableFrame frame, int format) {

        long numImgs = frame.getNumImages();
        for (int i = 0; i < numImgs; i++) {
            if (frame.getImage(i).getFormat() == format) {
                return frame.getImage(i);
            }//if
        }//for

        return null;
    }
}
