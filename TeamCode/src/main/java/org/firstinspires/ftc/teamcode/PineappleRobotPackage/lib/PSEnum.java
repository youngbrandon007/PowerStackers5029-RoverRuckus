package org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib;

/**
 * Created by ftcpi on 6/29/2017.
 */

<<<<<<< HEAD:TeamCode/src/main/java/org/firstinspires/ftc/teamcode/PineappleRobotPackage/lib/PineappleEnum.java
public class PineappleEnum {
=======
public class PSEnum {

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


>>>>>>> origin/master:TeamCode/src/main/java/org/firstinspires/ftc/teamcode/PineappleRobotPackage/lib/PSEnum.java
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

    }
    //Servo
    public enum ServoType{
        CONTINUOUS, LIMIT
    }
}

