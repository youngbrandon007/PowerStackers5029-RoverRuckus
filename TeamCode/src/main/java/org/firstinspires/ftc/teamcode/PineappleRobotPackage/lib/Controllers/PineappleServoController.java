package org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.Controllers;

import com.qualcomm.robotcore.hardware.ServoController;

import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PineappleController;
import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PineappleResources;

/**
 * Created by ftcpi on 8/7/2017.
 */

public class PineappleServoController extends PineappleController {
    public ServoController servoController;

    private PineappleResources resources;
    /**
     * Used to make a servo controller
     * @param name Hardware map name from the Sensor Handler
     * @param pineappleResources Resources passed through so that the makeController can hardware map properly
     */
    public PineappleServoController(String name, PineappleResources pineappleResources) {
        resources = pineappleResources;
        makeController(name, pineappleResources);
    }
    /**
     * Hard ware maps the servo controller
     * @param name Hardware map name
     * @param pineappleResources So that the method has access to the hardware map to register the sensor
     */
    @Override
    public void makeController(String name, PineappleResources pineappleResources) {
        controllerName = name;
        servoController = resources.hardwareMap.servoController.get(controllerName);

    }



}
