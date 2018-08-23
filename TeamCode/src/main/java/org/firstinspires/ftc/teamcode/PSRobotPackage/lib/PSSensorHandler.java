package org.firstinspires.ftc.teamcode.PSRobotPackage.lib;

import org.firstinspires.ftc.teamcode.PSRobotPackage.lib.Sensors.PSColorSensor;
import org.firstinspires.ftc.teamcode.PSRobotPackage.lib.Sensors.PSDistanceSensor;
import org.firstinspires.ftc.teamcode.PSRobotPackage.lib.Sensors.PSLightSensor;
import org.firstinspires.ftc.teamcode.PSRobotPackage.lib.Sensors.PSTouchSensor;

/**
 * Created by young on 8/7/2017.
 */

public class PSSensorHandler {

    private PSResources resources;

    public PSSensorHandler(PSResources pineappleResources){
        resources = pineappleResources;
    }

    public PSTouchSensor newTouchSensor(String name){
        PSTouchSensor sensor = new PSTouchSensor(name, resources);
        return sensor;
    }
    public PSLightSensor newLightSensor(String name){
        PSLightSensor sensor = new PSLightSensor(name, resources);
        return sensor;
    }

    public PSColorSensor newColorSensor(String name){
        PSColorSensor sensor = new PSColorSensor(name, resources);
        return sensor;
    }

    public PSDistanceSensor newDistanceSensor(String name){
        PSDistanceSensor sensor = new PSDistanceSensor(name, resources);
        return sensor;
    }
}
