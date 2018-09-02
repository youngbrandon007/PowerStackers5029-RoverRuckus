package org.firstinspires.ftc.teamcode.PSRobotLibs.lib;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.auto.PSAutoDrive;
import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.auto.PSSwitchBoard;
import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.drive.PSDrive;
import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.hardware.handlers.PSMotorHandler;
import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.hardware.handlers.PSServoHandler;

/**
 * Created by Brandon on 6/26/2017.
 *
 */

public class PSRobot {
    public PSMotorHandler motorHandler;
    public PSDrive drive;
    public PSAutoDrive auto;
    private PSResources resources;
    public PSSwitchBoard switchBoard;
    public PSServoHandler servoHandler;

    public PSRobot(OpMode opMode){
        resources = new PSResources(opMode);
        motorHandler = new PSMotorHandler(resources);
        drive = new PSDrive(resources);
        auto = new PSAutoDrive(resources, drive);
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


