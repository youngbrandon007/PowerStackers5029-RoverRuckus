package org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib;

/**
 * Created by Brandon on 10/21/2017.
 */

public class PineappleStaticFunction {
    public static double round(double val, int place){
        return Math.round(val*Math.pow(10, place))/Math.pow(10,place);
    }

    public static boolean isPositive(double value) {
        if (value >= 0) {
            return true;
        } else {
            return false;
        }
    }

    public static int distToCounts(double value, PineappleEnum.MotorValueType motorValueType, double wheelSize, double cpr) {
        switch (motorValueType) {
            case INCH:
                return (int) (cpr * (value / (PineappleRobotConstants.PI * wheelSize)));
            case DEGREES:
                return (int) (cpr * (value / 360));
            case CM:
                return (int) (cpr * ((value * PineappleRobotConstants.CMTOINCH) / (PineappleRobotConstants.PI * wheelSize)));
            case RADIANS:
                return (int) (cpr * (value / (2 * PineappleRobotConstants.PI)));
            case METER:
                return (int) (cpr * (((value*100) * PineappleRobotConstants.CMTOINCH) / (PineappleRobotConstants.PI * wheelSize)));
            case FEET:
                return (int) (cpr * ((value*12) / (PineappleRobotConstants.PI * wheelSize)));
            case YARDS:
                return (int) (cpr * ((value*36) / (PineappleRobotConstants.PI * wheelSize)));
            default:
                return 0;
        }
    }
}
