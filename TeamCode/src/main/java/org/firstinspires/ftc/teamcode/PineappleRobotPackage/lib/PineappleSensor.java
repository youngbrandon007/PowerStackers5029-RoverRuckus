package org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib;

/**
 * Created by young on 8/6/2017.
 */

public abstract class PineappleSensor {

    public String sensorName;
    
    public abstract void makeSensor(String name, PineappleResources pineappleResources);

    public abstract double getValue(PineappleEnum.PineappleSensorEnum pineappleSensorEnum);

}
