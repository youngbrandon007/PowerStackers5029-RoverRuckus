package org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.Auto.PSAutoDrive;
import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.Auto.PSSwitchBoard;
import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.Drive.PSDrive;

/**
 * Created by Brandon on 6/26/2017.
 *
 */

public class PSRobot {

    public PSMotorHandler motorHandler;

    public PSDrive drive;

    public PSAutoDrive auto;

    public PSSensorHandler sensorHandler;

    private PSResources resources;

    public PSSwitchBoard switchBoard;

    public PSServoHandler servoHandler;

    public PSRobot(OpMode LOM){
        resources = new PSResources(LOM);
        motorHandler = new PSMotorHandler(resources);
        drive = new PSDrive(resources);
        auto = new PSAutoDrive(resources, drive);
        sensorHandler = new PSSensorHandler(resources);
        servoHandler = new PSServoHandler(resources);
        switchBoard = new PSSwitchBoard(resources);
    }

    public void sayFeedBack(String objectName, double value){
        resources.feedBack.sayFeedBackWithOutUpdate(objectName, value);
    }
    public void sayFeedBack(String objectName, String value){
        resources.feedBack.sayFeedBackWithOutUpdate(objectName, value);
    }
    public void updateFeedBack(){
        resources.feedBack.update();
    }

}


