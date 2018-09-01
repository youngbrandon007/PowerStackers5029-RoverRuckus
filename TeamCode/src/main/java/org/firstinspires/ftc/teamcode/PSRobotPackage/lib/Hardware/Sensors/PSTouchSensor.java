package org.firstinspires.ftc.teamcode.PSRobotPackage.lib.Hardware.Sensors;

import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.PSRobotPackage.lib.PSResources;

/**
 * Created by ftcpi on 8/7/2017.
 */

public class PSTouchSensor {
    public TouchSensor touchSensor;
    public String name;
    private PSResources resources;


    public PSTouchSensor(String name, PSResources resources) {
        this.name = name;
        this.resources = resources;
        touchSensor = resources.hardwareMap.get(TouchSensor.class, name);
    }

    public boolean getState() {
        return touchSensor.isPressed();
    }
}
