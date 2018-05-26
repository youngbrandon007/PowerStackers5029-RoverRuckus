package org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.Controllers;

import com.qualcomm.robotcore.hardware.LegacyModule;

import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PineappleController;
import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PineappleResources;

/**
 * Created by ftcpi on 8/7/2017.
 */

public class PineappleLegacyModule extends PineappleController {
    public LegacyModule legacyModule;
    private PineappleResources resources;
    /**
     * Used to make a Legacy Module
     * @param name Hardware map name from the Sensor Handler
     * @param pineappleResources Resources passed through so that the makeController can hardware map properly
     */
    public PineappleLegacyModule(String name, PineappleResources pineappleResources) {
        resources = pineappleResources;
        makeController(name, pineappleResources);
    }
    /**
     * Hardware maps the controller
     * @param name Hardware map name
     * @param pineappleResources So that the method has access to the hardware map to register the sensor
     */
    @Override
    public void makeController(String name, PineappleResources pineappleResources) {
        controllerName = name;
        legacyModule = resources.hardwareMap.legacyModule.get(controllerName);
    }


}
