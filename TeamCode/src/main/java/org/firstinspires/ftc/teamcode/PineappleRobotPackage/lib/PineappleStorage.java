package org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

/**
 * Created by ftcpi on 6/29/2017.
 */

public class PineappleStorage {

    public HashMap<PineappleEnum.MotorLoc, PineappleMotor> driveMotors = new HashMap<PineappleEnum.MotorLoc, PineappleMotor>();

    public HashMap<String, PineappleMotor> motors = new HashMap<String, PineappleMotor>();

    public void insert(PineappleMotor motor){
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

    public ArrayList<PineappleMotor> getDrivemotors(PineappleEnum.MotorLoc motorLoc){
        ArrayList<PineappleMotor> returnMotors = new ArrayList<PineappleMotor>();
        for (Map.Entry<PineappleEnum.MotorLoc, PineappleMotor> entry : driveMotors.entrySet()) {
            PineappleEnum.MotorLoc loc = entry.getKey();
            PineappleMotor motor = entry.getValue();
            if(loc == motorLoc){
                returnMotors.add(motor);
            }


        }
        return returnMotors;
    }
    public ArrayList<PineappleMotor> getDrivemotors(){
        ArrayList<PineappleMotor> returnMotors = new ArrayList<PineappleMotor>();
        for (Map.Entry<PineappleEnum.MotorLoc, PineappleMotor> entry : driveMotors.entrySet()) {
            PineappleEnum.MotorLoc loc = entry.getKey();
            PineappleMotor motor = entry.getValue();
            if(loc != PineappleEnum.MotorLoc.NONE){
                returnMotors.add(motor);
            }


        }
        return returnMotors;
    }
}
