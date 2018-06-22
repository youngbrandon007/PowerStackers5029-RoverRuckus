package org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.Sensors;

import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PSEnum;
import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PSResources;
import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PSSensor;

/**
 * Created by ftcpi on 8/7/2017.
 */

public class PSTouchSensor extends PSSensor {
    public TouchSensor touchSensor;
    private PSResources resources;
    public PSTouchSensor(String name, PSResources pineappleResources){
        resources = pineappleResources;
        makeSensor(name, pineappleResources);
    }
    @Override
    public void makeSensor(String name, PSResources pineappleResources) {
        sensorName = name;
        touchSensor = resources.hardwareMap.touchSensor.get(sensorName);
    }

    @Override
    public double getValue(PSEnum.PineappleSensorEnum sensor) {
        if (touchSensor.isPressed()) {
            return 1;
        } else{
            return 0;
        }

    }
}
