package org.firstinspires.ftc.teamcode.PSRobotPackage.lib.Sensors;

import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.PSRobotPackage.lib.PSResources;

/**
 * Created by ftcpi on 8/7/2017.
 */

public class PSDistanceSensor {
    public DistanceSensor distanceSensor;
    public String name;
    private PSResources resources;


    public PSDistanceSensor(String name, PSResources resources) {
        this.name = name;
        this.resources = resources;
        distanceSensor = resources.hardwareMap.get(DistanceSensor.class, name);
    }

    public double getDistance() {
        return distanceSensor.getDistance(DistanceUnit.INCH);
    }
}
