package org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.Sensors;

import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PSEnum;
import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PSResources;
import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PSSensor;

/**
 * Created by ftcpi on 8/7/2017.
 */

public class PSOpticalDistanceSensor extends PSSensor {
    public OpticalDistanceSensor opticalDistanceSensor;
    private PSResources resources;
    public PSOpticalDistanceSensor(String name, PSResources pineappleResources){
        resources = pineappleResources;
        makeSensor(name, pineappleResources);
    }

    @Override
    public void makeSensor(String name, PSResources pineappleResources) {
        sensorName = name;
        opticalDistanceSensor = resources.hardwareMap.opticalDistanceSensor.get(sensorName);
    }

    public void ODSLEDToggle(boolean toggle) {
        opticalDistanceSensor.enableLed(toggle);
    }

    public double getValue(PSEnum.PineappleSensorEnum pineappleSensorEnum) {
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
