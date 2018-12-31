package org.firstinspires.ftc.teamcode.roverRuckus.Robot_r4.Paths;

import android.os.Environment;

import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.util.ArrayList;

public class ConstantsLoader {

    public static DriveConstraints getDriveConstraints(){
        String[] data = readSettings();
        return new DriveConstraints(Double.valueOf(data[0]), Double.valueOf(data[1]), Double.valueOf(data[2]), Double.valueOf(data[3]));
    }

    public static double[] getMotionPIDVA(){
        String[] data = readSettings();
        return new double[]{Double.valueOf(data[4]), Double.valueOf(data[5]), Double.valueOf(data[6]), Double.valueOf(data[7]), Double.valueOf(data[8])};
    }

    public static double[] getRotationPIDVA(){
        String[] data = readSettings();
        return new double[]{Double.valueOf(data[9]), Double.valueOf(data[10]), Double.valueOf(data[11]), Double.valueOf(data[12]), Double.valueOf(data[13])};
    }

    public static String getPath(){
        return Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_DOWNLOADS) + "/auto/settings.txt";
    }

    public static String[] readSettings(){
        return readFile(getPath());
    }

    private static String[] readFile(String path){
        try {
            BufferedReader br = new BufferedReader(new FileReader(new File(path)));
            String line = br.readLine();
            ArrayList<String> output = new ArrayList<>();
            while (line != null) {
                if(!line.equals("")) output.add(line);
                line = br.readLine();
            }
            return output.toArray(new String[]{});
        }catch(Exception e){
            e.printStackTrace();
            return null;
        }
    }
}
