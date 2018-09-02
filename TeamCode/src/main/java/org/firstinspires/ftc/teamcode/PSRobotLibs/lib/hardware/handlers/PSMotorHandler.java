package org.firstinspires.ftc.teamcode.PSRobotLibs.lib.hardware.handlers;

import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.hardware.PSMotor;
import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.PSEnum;
import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.PSResources;

/**
 * Created by Brandon on 6/26/2017.
 */

public class PSMotorHandler {

    /**
     * resources for motor
     */
    private PSResources resources;

    /**
     * Motor handler constructor for the robot
     *
     * @param r
     */
    public PSMotorHandler(PSResources r) {
        resources = r;
    }


    public PSMotor newMotor(String name, double gearRatio) {
        resources.storage.insert(new PSMotor(resources, name, -1, 1, 0, 1, false, true, PSEnum.MotorLoc.NONE, gearRatio));
        return getMotor(name);
    }
    public PSMotor newMotor(String name, double scale, boolean exp, boolean deadArea, double gearRatio) {
        PSMotor motor = new PSMotor(resources, name, -1, 1, 0, scale, exp, deadArea, PSEnum.MotorLoc.NONE, gearRatio);
        resources.storage.insert(motor);
        return  motor;
    }

    /**
     * PowerStacker motor constructor
     *
     * @param name name of the motor for hardwaremapping
     * @param powerMin the min power for the motor
     * @param powerMax the max power for the motor
     * @param powerDefault the default power for the motor
     * @param scale the scale for the motor
     * @param exp whether the motor should be exp scaled
     * @param deadArea whether the motor uses dead area
     * @param gearRatio the gear ratio of the motor from the bare motor to the drive part
     */
    public PSMotor newMotor(String name, double powerMin, double powerMax, double powerDefault, double scale, boolean exp, boolean deadArea, double gearRatio) {
        resources.storage.insert(new PSMotor(resources, name, powerMin, powerMax, powerDefault, scale, exp, deadArea, PSEnum.MotorLoc.NONE,  gearRatio));
        return getMotor(name);
    }


    //With Motor location
    public PSMotor newDriveMotor(String name, PSEnum.MotorLoc motorLoc, double gearRatio) {
        resources.storage.insert(new PSMotor(resources, name, -1, 1, 0, 1, false, true, motorLoc , gearRatio));
        return getMotor(name);
    }

    public PSMotor newDriveMotor(String name, double scale, boolean exp, boolean deadArea, PSEnum.MotorLoc motorLoc, double gearRatio) {
        resources.storage.insert(new PSMotor(resources, name, -1, 1, 0, scale, exp, deadArea, motorLoc,  gearRatio));
        return getMotor(name);
    }

    public PSMotor newDriveMotor(String name, double powerMin, double powerMax, double powerDefault, double scale, boolean exp, boolean deadArea, PSEnum.MotorLoc motorLoc, double gearRatio) {
        resources.storage.insert(new PSMotor(resources, name, powerMin, powerMax, powerDefault, scale, exp, deadArea, motorLoc, gearRatio));
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