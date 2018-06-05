package org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by Brandon on 7/12/2017.
 */

//This will be added after testing of the motorObject handler and will control all local handlers that are needed by many classes like
public class PineappleResources {

    /////////
    //LOCAL//
    /////////

    public PineappleStorage storage;

    public PineappleFeedBack feedBack;

    public OpMode opMode;

    public Telemetry telemetry;

    public HardwareMap hardwareMap;

    PineappleResources(OpMode LOM){
        storage = new PineappleStorage();
        opMode = LOM;
        telemetry = LOM.telemetry;
        feedBack = new PineappleFeedBack(telemetry);
        hardwareMap = LOM.hardwareMap;
    }
}
