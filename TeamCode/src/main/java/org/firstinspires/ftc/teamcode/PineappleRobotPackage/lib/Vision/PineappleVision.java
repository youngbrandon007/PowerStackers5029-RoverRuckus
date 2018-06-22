package org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.Vision;

import android.graphics.Bitmap;
import android.os.Environment;
import android.support.annotation.Nullable;

import com.vuforia.Image;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.opencv.android.Utils;
import org.opencv.core.Mat;

import java.io.File;
import java.io.FileOutputStream;


/**
 * Created by young on 9/13/2017.
 */

public class PineappleVision {
    //    public final static Scalar blueLow = new Scalar(108, 0 , 220);
//    public final static Scalar blueLow = new Scalar(220, 99, 45);
//
//    public final static Scalar blueHigh = new Scalar(192, 84, 100);
    //    public final static Scalar blueHigh = new Scalar(178, 255 , 255);


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
