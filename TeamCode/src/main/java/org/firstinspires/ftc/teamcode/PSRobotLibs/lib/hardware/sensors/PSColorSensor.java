package org.firstinspires.ftc.teamcode.PSRobotLibs.lib.hardware.sensors;

import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.PSResources;

/**
 * Created by ftcpi on 8/7/2017.
 */

public class PSColorSensor{
    public ColorSensor colorSensor;
    public String name;
    private PSResources resources;


    public PSColorSensor(String name, PSResources resources){
        this.name = name;
        this.resources = resources;
        colorSensor = resources.hardwareMap.get(ColorSensor.class, name);
    }

    public double getRed(){
        return colorSensor.red();
    }

    public double getGreen(){
        return colorSensor.green();
    }

    public double getBlue() {
        return colorSensor.blue();
    }

    public double getAlpha(){
        return colorSensor.alpha();
    }
}
