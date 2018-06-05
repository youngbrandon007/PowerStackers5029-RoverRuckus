package org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib;

/**
 * Created by ftcpi on 6/29/2017.
 */

public class PineappleEnum {

    //Year specific Enums
    public enum AllianceColor {
        RED, BLUE
    }
    public enum VuMarkLocation {
        UNKNOWN, CENTER, RIGHT, LEFT
    }

    public enum JewelState {
        //Left to right
        //NON = nothing
        BLUE_RED, RED_BLUE, NON_BLUE, NON_RED, BLUE_NON, RED_NON, NON_NON
    }


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




    //Sensors
    public enum PineappleSensorEnum {
        TOUCH, ODSRAWMAX, ODSRAW, ODSLIGHTDETECTED, CSRED, CSGREEN, CSBLUE, CSAVG, CSALPHA, CSARGB,CSLEDON, CSLEDOFF, GSX, GSY, GSZ, GSHEADING, GSROTATIONFRACTION, US
    }

    public enum Condition {
        EQUAL, LESSTHAN, GREATERTHAN
    }

    //Servo
    public enum ServoType{
        CONTINUOUS, LIMIT
    }


    //Drive Type
    public enum DriveType {
        TANK, MECANUM
    }

}

