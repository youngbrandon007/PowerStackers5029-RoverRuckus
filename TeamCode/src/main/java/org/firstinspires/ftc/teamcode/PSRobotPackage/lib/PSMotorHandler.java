package org.firstinspires.ftc.teamcode.PSRobotPackage.lib;

/**
 * Created by Brandon on 6/26/2017.
 */

public class PSMotorHandler {

    private PSResources resources;

    public PSMotorHandler(PSResources r) {
        resources = r;
    }

    //Without Enum input
    public PSMotor newMotor(String name) {
        resources.storage.insert(new PSMotor(resources, name, -1, 1, 0, 1, false, true, PSEnum.MotorLoc.NONE, PSEnum.MotorType.UNDI));
        return getMotor(name);
    }
    public PSMotor newMotor(String name, PSEnum.MotorType motorType) {
        resources.storage.insert(new PSMotor(resources, name, -1, 1, 0, 1, false, true, PSEnum.MotorLoc.NONE, motorType));
        return getMotor(name);
    }

    public PSMotor newMotor(String name, double scale, boolean exp, boolean deadArea, PSEnum.MotorType motorType) {
        PSMotor motor = new PSMotor(resources, name, -1, 1, 0, scale, exp, deadArea, PSEnum.MotorLoc.NONE, motorType);
        resources.storage.insert(motor);


        return  motor;
    }

    public PSMotor newMotor(String name, double powerMin, double powerMax, double powerDefault, double scale, boolean exp, boolean deadArea, PSEnum.MotorType motorType) {
        resources.storage.insert(new PSMotor(resources, name, powerMin, powerMax, powerDefault, scale, exp, deadArea, PSEnum.MotorLoc.NONE, motorType));
        return getMotor(name);
    }


    //With enum input
    public PSMotor newDriveMotor(String name, PSEnum.MotorLoc motorLoc) {
        resources.storage.insert(new PSMotor(resources, name, -1, 1, 0, 1, false, true, motorLoc, PSEnum.MotorType.UNDI));
        return getMotor(name);
    }
    public PSMotor newDriveMotor(String name, PSEnum.MotorLoc motorLoc, PSEnum.MotorType motorType) {
        resources.storage.insert(new PSMotor(resources, name, -1, 1, 0, 1, false, true, motorLoc, motorType));
        return getMotor(name);
    }

    public PSMotor newDriveMotor(String name, double scale, boolean exp, boolean deadArea, PSEnum.MotorLoc motorLoc, PSEnum.MotorType motorType) {
        resources.storage.insert(new PSMotor(resources, name, -1, 1, 0, scale, exp, deadArea, motorLoc, motorType));
        return getMotor(name);
    }

    public PSMotor newDriveMotor(String name, double powerMin, double powerMax, double powerDefault, double scale, boolean exp, boolean deadArea, PSEnum.MotorLoc motorLoc, PSEnum.MotorType motorType) {
        resources.storage.insert(new PSMotor(resources, name, powerMin, powerMax, powerDefault, scale, exp, deadArea, motorLoc, motorType));
        return getMotor(name);
    }

    public PSMotor getMotor(String name) {
        if (!resources.storage.motors.containsKey(name)) {
            return null;
        } else {
            return resources.storage.motors.get(name);
        }
    }

}