package org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.Sensors;

import com.qualcomm.robotcore.hardware.UltrasonicSensor;

import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PineappleEnum;
import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PineappleResources;
import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PineappleSensor;

/**
 * Created by ftcpi on 8/7/2017.
 */

public class PineappleUltrasonicSensor extends PineappleSensor {
    public UltrasonicSensor ultrasonicSensor;
    private PineappleResources resources;
    public PineappleUltrasonicSensor (String name, PineappleResources pineappleResources){
        resources = pineappleResources;
        makeSensor(name, pineappleResources);
    }

    @Override
    public void makeSensor(String name, PineappleResources pineappleResources) {
        sensorName = name;
        ultrasonicSensor = resources.hardwareMap.ultrasonicSensor.get(sensorName);
    }

    public double getValue( PineappleEnum.PineappleSensorEnum sensorEnum) {
        return ultrasonicSensor.getUltrasonicLevel();

    }
}
