package org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.Sensors;

import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PineappleEnum;
import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PineappleResources;
import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PineappleSensor;

/**
 * Created by ftcpi on 8/7/2017.
 */

public class PineappleTouchSensor extends PineappleSensor {
    public TouchSensor  touchSensor;
    private PineappleResources resources;
    public PineappleTouchSensor (String name, PineappleResources pineappleResources){
        resources = pineappleResources;
        makeSensor(name, pineappleResources);
    }
    @Override
    public void makeSensor(String name, PineappleResources pineappleResources) {
        sensorName = name;
        touchSensor = resources.hardwareMap.touchSensor.get(sensorName);
    }

    @Override
    public double getValue(PineappleEnum.PineappleSensorEnum sensor) {
        if (touchSensor.isPressed()) {
            return 1;
        } else{
            return 0;
        }

    }
}
