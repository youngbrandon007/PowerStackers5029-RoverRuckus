package org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib;

/**
 * Created by ftcpi on 6/29/2017.
 */


public class PSEnum {

    //Motors
    public enum MotorLoc {
        RIGHT, LEFT, NONE, RIGHTFRONT, LEFTFRONT, RIGHTBACK, LEFTBACK
    }

    public enum MotorValueType {
        INCH, COUNTS, DEGREES, CM, RADIANS, METER, FEET, YARDS
    }

    public enum MotorType {
        NEV60, NEV40, NEV20, NEV3_7, UNDI
    }

    public enum SensorType{
        Touch, Light, Color, Distance
    }
    //Servo
    public enum ServoType{
        CONTINUOUS, LIMIT
    }
}

