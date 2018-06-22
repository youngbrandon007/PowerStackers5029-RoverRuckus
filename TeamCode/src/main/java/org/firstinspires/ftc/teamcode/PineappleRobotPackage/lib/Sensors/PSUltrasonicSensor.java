package org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.Sensors;

import com.qualcomm.robotcore.hardware.UltrasonicSensor;

import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PSEnum;
import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PSResources;
import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PSSensor;

/**
 * Created by ftcpi on 8/7/2017.
 */

public class PSUltrasonicSensor extends PSSensor {
    public UltrasonicSensor ultrasonicSensor;
    private PSResources resources;
    public PSUltrasonicSensor(String name, PSResources pineappleResources){
        resources = pineappleResources;
        makeSensor(name, pineappleResources);
    }

    @Override
    public void makeSensor(String name, PSResources pineappleResources) {
        sensorName = name;
        ultrasonicSensor = resources.hardwareMap.ultrasonicSensor.get(sensorName);
    }

    public double getValue( PSEnum.PineappleSensorEnum sensorEnum) {
        return ultrasonicSensor.getUltrasonicLevel();

    }
}
