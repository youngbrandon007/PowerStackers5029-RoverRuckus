package org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.Sensors;

import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PineappleEnum;
import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PineappleResources;
import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PineappleSensor;

/**
 * Created by ftcpi on 8/7/2017.
 */

public class PineappleColorSensor extends PineappleSensor {
    public ColorSensor colorSensor;
    private PineappleResources resources;

    public PineappleColorSensor(String name, PineappleResources pineappleResources) {
        resources = pineappleResources;
        makeSensor(name, pineappleResources);
    }

    @Override
    public void makeSensor(String name, PineappleResources pineappleResources) {
        sensorName = name;
        colorSensor = resources.hardwareMap.colorSensor.get(sensorName);
    }

    public void CSLEDToggle(boolean toggle) {
        colorSensor.enableLed(toggle);
    }

    @Override
    public double getValue(PineappleEnum.PineappleSensorEnum pineappleSensorEnum) {
        switch (pineappleSensorEnum) {
            case CSBLUE:
                return (double) colorSensor.blue();
            case CSGREEN:
                return (double) colorSensor.green();
            case CSRED:
                return (double) colorSensor.red();
            case CSALPHA:
                return (double) colorSensor.alpha();
            case CSARGB:
                return (double) colorSensor.argb();
            case CSAVG:
                return (double) (colorSensor.red() + colorSensor.green() + colorSensor.blue()) / 3;
            case CSLEDON:
                CSLEDToggle(true);
                return 0;
            case CSLEDOFF:
                CSLEDToggle(false);
                return 0;
            default:
                return 0;

        }
    }


}
