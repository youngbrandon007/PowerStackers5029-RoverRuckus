package org.firstinspires.ftc.teamcode.PSRobotPackage.lib;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by Brandon on 7/12/2017.
 */

//This will be added after testing of the motorObject handler and will control all local handlers that are needed by many classes like
public class PSResources {

    /////////
    //LOCAL//
    /////////

    public PSStorage storage;

    public PSFeedBack feedBack;

    public OpMode opMode;

    public Telemetry telemetry;

    public HardwareMap hardwareMap;

    PSResources(OpMode LOM){
        storage = new PSStorage();
        opMode = LOM;
        telemetry = LOM.telemetry;
        feedBack = new PSFeedBack(telemetry);
        hardwareMap = LOM.hardwareMap;
    }
}
