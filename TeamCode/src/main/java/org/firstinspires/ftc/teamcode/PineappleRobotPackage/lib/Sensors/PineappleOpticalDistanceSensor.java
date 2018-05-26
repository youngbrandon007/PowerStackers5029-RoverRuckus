package org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.Sensors;

import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PineappleEnum;
import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PineappleResources;
import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PineappleSensor;

/**
 * Created by ftcpi on 8/7/2017.
 */

public class PineappleOpticalDistanceSensor extends PineappleSensor {
    public OpticalDistanceSensor opticalDistanceSensor;
    private PineappleResources resources;
    public PineappleOpticalDistanceSensor (String name, PineappleResources pineappleResources){
        resources = pineappleResources;
        makeSensor(name, pineappleResources);
    }

    @Override
    public void makeSensor(String name, PineappleResources pineappleResources) {
        sensorName = name;
        opticalDistanceSensor = resources.hardwareMap.opticalDistanceSensor.get(sensorName);
    }

    public void ODSLEDToggle(boolean toggle) {
        opticalDistanceSensor.enableLed(toggle);
    }

    public double getValue(PineappleEnum.PineappleSensorEnum pineappleSensorEnum) {
        switch (pineappleSensorEnum) {
            case ODSRAWMAX:
                return opticalDistanceSensor.getRawLightDetectedMax();
            case ODSRAW:
                return opticalDistanceSensor.getRawLightDetected();
            case ODSLIGHTDETECTED:
                return opticalDistanceSensor.getLightDetected();
            default:
                return 0;
        }

    }
}
