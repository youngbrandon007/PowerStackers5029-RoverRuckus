package org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

/**
 * Created by ftcpi on 6/29/2017.
 */

public class PSStorage {

    public HashMap<PSEnum.MotorLoc, PSMotor> driveMotors = new HashMap<PSEnum.MotorLoc, PSMotor>();

    public HashMap<String, PSMotor> motors = new HashMap<String, PSMotor>();

    public void insert(PSMotor motor){
        switch (motor.motorLoc) {
            case RIGHT:
                driveMotors.put(motor.motorLoc, motor);
                break;
            case LEFT:
                driveMotors.put(motor.motorLoc, motor);
                break;
            case RIGHTFRONT:
                driveMotors.put(motor.motorLoc, motor);
                break;
            case LEFTFRONT:
                driveMotors.put(motor.motorLoc, motor);
                break;
            case RIGHTBACK:
                driveMotors.put(motor.motorLoc, motor);
                break;
            case LEFTBACK:
                driveMotors.put(motor.motorLoc, motor);
                break;
        }
        motors.put(motor.motorName, motor);
    }

    public ArrayList<PSMotor> getDrivemotors(PSEnum.MotorLoc motorLoc){
        ArrayList<PSMotor> returnMotors = new ArrayList<PSMotor>();
        for (Map.Entry<PSEnum.MotorLoc, PSMotor> entry : driveMotors.entrySet()) {
            PSEnum.MotorLoc loc = entry.getKey();
            PSMotor motor = entry.getValue();
            if(loc == motorLoc){
                returnMotors.add(motor);
            }


        }
        return returnMotors;
    }
    public ArrayList<PSMotor> getDrivemotors(){
        ArrayList<PSMotor> returnMotors = new ArrayList<PSMotor>();
        for (Map.Entry<PSEnum.MotorLoc, PSMotor> entry : driveMotors.entrySet()) {
            PSEnum.MotorLoc loc = entry.getKey();
            PSMotor motor = entry.getValue();
            if(loc != PSEnum.MotorLoc.NONE){
                returnMotors.add(motor);
            }


        }
        return returnMotors;
    }
}
