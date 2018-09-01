package org.firstinspires.ftc.teamcode.PSRobotLibs.lib;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.PSRobotLibs.PSSettings;

/**
 * Created by ftcpi on 6/29/2017.
 */

public class PSFeedBack {

    public boolean giveFeedBack = true;

    public Telemetry telemetry;

    public PSFeedBack(Telemetry t){
        telemetry = t;
        giveFeedBack = PSSettings.feedBack;
    }

    public void sayFeedBack(String objectName, double amount){
        if(giveFeedBack){
            telemetry.addData(objectName, amount);
            telemetry.update();
        }
    }

    public void sayFeedBackWithOutUpdate(String objectName, double amount){
        if(giveFeedBack){
            telemetry.addData(objectName, amount);
        }
    }
    public void sayFeedBackWithOutUpdate(String objectName, String amount){
        if(giveFeedBack){
            telemetry.addData(objectName, amount);
        }
    }
    public void update(){
        telemetry.update();
    }
}

