package org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.Controllers;

import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;

import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PineappleController;
import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PineappleResources;

/**
 * Created by ftcpi on 8/7/2017.
 */

public class PineappleDeviceInterfaceModule extends PineappleController {
    public DeviceInterfaceModule deviceInterfaceModule;
    private PineappleResources resources;

    /**
     * Used to make a CDMI
     * @param name Hardware map name from the Sensor Handler
     * @param pineappleResources Resources passed through so that the makeController can hardware map properly
     */
    public PineappleDeviceInterfaceModule(String name, PineappleResources pineappleResources) {
        resources = pineappleResources;
        makeController(name, pineappleResources);
    }

    /**
     * Hardware maps the CDIM
     * @param name Hardware map name
     * @param pineappleResources So that the method has access to the hardware map to register the sensor
     */
    @Override
    public void makeController(String name, PineappleResources pineappleResources) {
        controllerName = name;
        deviceInterfaceModule = resources.hardwareMap.deviceInterfaceModule.get(controllerName);
    }


}
