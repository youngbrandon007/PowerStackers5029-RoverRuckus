package org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib;

import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.Sensors.PSColorSensor;
import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.Sensors.PSGyroSensor;
import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.Sensors.PSOpticalDistanceSensor;
import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.Sensors.PSTouchSensor;
import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.Sensors.PSUltrasonicSensor;

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
    public PSUltrasonicSensor newUltrasonicSensor(String name){
        PSUltrasonicSensor sensor = new PSUltrasonicSensor(name, resources);
        return sensor;
    }
    public PSOpticalDistanceSensor newOpticalDistanceSensor(String name){
        PSOpticalDistanceSensor sensor = new PSOpticalDistanceSensor(name, resources);
        return sensor;
    }

    public PSColorSensor newColorSensor(String name){
        PSColorSensor sensor = new PSColorSensor(name, resources);
        return sensor;
    }

    public PSGyroSensor newGyroSensor(String name){
        PSGyroSensor sensor = new PSGyroSensor(name, resources);
        return sensor;
    }
}
