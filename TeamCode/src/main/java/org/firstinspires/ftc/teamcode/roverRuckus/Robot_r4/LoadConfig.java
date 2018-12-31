package org.firstinspires.ftc.teamcode.roverRuckus.Robot_r4;

import android.os.Environment;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.util.ArrayList;

class LoadConfig {

    static String getConfig(){
        String data = loadfile()[0];
        if(data.contains(":"))
            return data.split(":")[1];
        else
            return null;
    }

    private static String getDir(String file){
        return Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_DOWNLOADS) + "/auto/" + file;
    }

    public static String getDataPath(){
        return getDir("data.txt");
    }


    private static String[] loadfile(){
        try {
            BufferedReader br = new BufferedReader(new FileReader(new File(getDataPath())));
            StringBuilder sb = new StringBuilder();
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
