package org.firstinspires.ftc.teamcode.RelicRecoveryWorlds;

import android.os.Environment;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;

/**
 * Created by ftcpi on 2/7/2018.
 */
@Autonomous(name = "PID")
public class WorldPIDTesting extends WorldConfig {



    private void writeToFile(String name, String data) {
        File path = Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_DOWNLOADS);
        File file = new File(path + "/PID", name + ".csv");
        try {
            FileOutputStream stream = new FileOutputStream(file, true);
            stream.write(data.getBytes());
            stream.close();
            Log.i("saveData", "Data Saved");
        } catch (IOException e) {
            Log.e("SAVE DATA", "Could not write file " + e.getMessage());
        }
    }

    @Override
    public void init() {

    }

    @Override
    public void loop() {

    }
}
