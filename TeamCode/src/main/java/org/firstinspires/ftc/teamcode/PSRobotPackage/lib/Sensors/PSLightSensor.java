package org.firstinspires.ftc.teamcode.PSRobotPackage.lib.Sensors;

import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

import org.firstinspires.ftc.teamcode.PSRobotPackage.lib.PSResources;

/**
 * Created by ftcpi on 8/7/2017.
 */

public class PSLightSensor {
    public OpticalDistanceSensor opticalDistanceSensor;
    public String name;
    private PSResources resources;


    public PSLightSensor(String name, PSResources resources) {
        this.name = name;
        this.resources = resources;
        opticalDistanceSensor = resources.hardwareMap.get(OpticalDistanceSensor.class, name);
    }

    public double getLight() {
        return opticalDistanceSensor.getLightDetected();
    }

    public double getRawLight() {
        return opticalDistanceSensor.getRawLightDetected();
    }
}
