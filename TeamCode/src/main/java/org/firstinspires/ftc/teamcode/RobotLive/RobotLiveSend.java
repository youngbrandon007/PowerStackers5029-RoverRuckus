package org.firstinspires.ftc.teamcode.RobotLive;

import java.io.BufferedReader;
import java.io.File;
import java.io.IOException;
import java.io.InputStreamReader;
import java.net.URL;
import java.net.URLConnection;


public class RobotLiveSend {
    public RobotLiveSend() {
    }

    public static RobotLiveData createNewRun(String ip) {
        if (!ip.endsWith("/")) {
            ip = ip + "/";
        }

        try {
            URL connect = new URL(ip + "video?action=create");
            URLConnection yc = connect.openConnection();
            BufferedReader in = new BufferedReader(new InputStreamReader(yc.getInputStream()));
            int rin = Integer.parseInt(in.readLine());
            in.close();
            return new RobotLiveData(rin);
        } catch (IOException var5) {
            var5.printStackTrace();
            return null;
        }
    }

    public static String send(RobotLiveData data, String ip) {
        if (!ip.endsWith("/")) {
            ip = ip + "/";
        }
        if(data == null){
            return "RobotLive.DataSend - null round number";
        }

        try {
            MultipartUtility multipart = new MultipartUtility(ip + "r", "UTF-8");
            multipart.addFormField("RIN", "I:" + String.valueOf(data.RIN));
            int i;
            if (data.dataNames.size() != 0) {
                for(i = 0; i < data.dataNames.size(); ++i) {
                    multipart.addFormField((String)data.dataNames.get(i), (String)data.data.get(i));
                }
            }

            if (data.live != null) {
                multipart.addBitmap("LIVE", data.live);
            }

            if (data.files.size() != 0) {
                for(i = 0; i < data.fileNames.size(); ++i) {
                    multipart.addFilePart((String)data.fileNames.get(i), (File)data.files.get(i));
                }
            }

            multipart.finish();
            data.reset();

            return "RobotLive.DataSend - success";
        } catch (IOException var4) {
            var4.printStackTrace();
            return "RobotLive.DataSend - error sending data";
        }

    }

}

