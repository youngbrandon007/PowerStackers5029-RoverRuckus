package org.firstinspires.ftc.teamcode.PSRobotLibs.lib.utils;

import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.PSEnum;
import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.PSRobotConstants;

import java.util.HashMap;

/**
 * Created by Brandon on 10/21/2017.
 */

public class PSGeneralUtils {
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
    public static HashMap arraysToHashMap(String[] s, double[] d){
        HashMap<String, Double> hash = new HashMap<>();
        for (int x = 0; x< s.length; x++){
            hash.put(s[x], d[x]);
        }
        return hash;
    }
    public static int distToCounts(double value, PSEnum.MotorValueType motorValueType, double wheelSize, double cpr) {
        switch (motorValueType) {
            case INCH:
                return (int) (cpr * (value / (PSRobotConstants.PI * wheelSize)));
            case DEGREES:
                return (int) (cpr * (value / 360));
            case CM:
                return (int) (cpr * ((value * PSRobotConstants.CMTOINCH) / (PSRobotConstants.PI * wheelSize)));
            case RADIANS:
                return (int) (cpr * (value / (2 * PSRobotConstants.PI)));
            case METER:
                return (int) (cpr * (((value*100) * PSRobotConstants.CMTOINCH) / (PSRobotConstants.PI * wheelSize)));
            case FEET:
                return (int) (cpr * ((value*12) / (PSRobotConstants.PI * wheelSize)));
            case YARDS:
                return (int) (cpr * ((value*36) / (PSRobotConstants.PI * wheelSize)));
            default:
                return 0;
        }
    }
}
