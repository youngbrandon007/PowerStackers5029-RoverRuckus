package org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib;

import java.util.Map;

/**
 * Created by Brandon on 6/26/2017.
 */

public class PineappleMotorHandler {

    private PineappleResources resources;

    public PineappleMotorHandler(PineappleResources r) {
        resources = r;
    }

    //Without Enum input
    public PineappleMotor newMotor(String name) {
        resources.storage.insert(new PineappleMotor(resources, name, -1, 1, 0, 1, false, true, PineappleEnum.MotorLoc.NONE, PineappleEnum.MotorType.UNDI));
        return getMotor(name);
    }
    public PineappleMotor newMotor(String name, PineappleEnum.MotorType motorType) {
        resources.storage.insert(new PineappleMotor(resources, name, -1, 1, 0, 1, false, true, PineappleEnum.MotorLoc.NONE, motorType));
        return getMotor(name);
    }

    public PineappleMotor newMotor(String name, double scale, boolean exp, boolean deadArea, PineappleEnum.MotorType motorType) {
        PineappleMotor motor = new PineappleMotor(resources, name, -1, 1, 0, scale, exp, deadArea, PineappleEnum.MotorLoc.NONE, motorType);
        resources.storage.insert(motor);


        return  motor;
    }

    public PineappleMotor newMotor(String name, double powerMin, double powerMax, double powerDefault, double scale, boolean exp, boolean deadArea, PineappleEnum.MotorType motorType) {
        resources.storage.insert(new PineappleMotor(resources, name, powerMin, powerMax, powerDefault, scale, exp, deadArea, PineappleEnum.MotorLoc.NONE, motorType));
        return getMotor(name);
    }


    //With enum input
    public PineappleMotor newDriveMotor(String name, PineappleEnum.MotorLoc motorLoc) {
        resources.storage.insert(new PineappleMotor(resources, name, -1, 1, 0, 1, false, true, motorLoc, PineappleEnum.MotorType.UNDI));
        return getMotor(name);
    }
    public PineappleMotor newDriveMotor(String name, PineappleEnum.MotorLoc motorLoc, PineappleEnum.MotorType motorType) {
        resources.storage.insert(new PineappleMotor(resources, name, -1, 1, 0, 1, false, true, motorLoc, motorType));
        return getMotor(name);
    }

    public PineappleMotor newDriveMotor(String name, double scale, boolean exp, boolean deadArea, PineappleEnum.MotorLoc motorLoc, PineappleEnum.MotorType motorType) {
        resources.storage.insert(new PineappleMotor(resources, name, -1, 1, 0, scale, exp, deadArea, motorLoc, motorType));
        return getMotor(name);
    }

    public PineappleMotor newDriveMotor(String name, double powerMin, double powerMax, double powerDefault, double scale, boolean exp, boolean deadArea, PineappleEnum.MotorLoc motorLoc, PineappleEnum.MotorType motorType) {
        resources.storage.insert(new PineappleMotor(resources, name, powerMin, powerMax, powerDefault, scale, exp, deadArea, motorLoc, motorType));
        return getMotor(name);
    }

    public PineappleMotor getMotor(String name) {
        if (!resources.storage.motors.containsKey(name)) {
            return null;
        } else {
            return resources.storage.motors.get(name);
        }
    }

}