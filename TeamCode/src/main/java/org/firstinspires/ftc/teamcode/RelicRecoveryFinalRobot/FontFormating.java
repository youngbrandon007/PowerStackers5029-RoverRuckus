package org.firstinspires.ftc.teamcode.RelicRecoveryFinalRobot;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by young on 1/10/2018.
 */

public class FontFormating {
    public static final String checkMark = "✓";
    public static final String xMark = "x";
    public static final String emptyBox = "□";
    public static final String filledBox = "■";
    public static final String[] bigCheckMark =
            {
                    "                  ▗ " ,
                            "                ▄▛▀ " ,
                            "              ▄█▘   " ,
                            "            ▄█▀     " ,
                            "          ▗▟▛▘      " ,
                            "         ▗█▛        " ,
                            " ▗▄▖    ▟█▘         " ,
                            " ▀██▖  ▟█▘          " ,
                            "  ▝██▖▟█▘           " ,
                            "   ▐███▌            " ,
                            "    ▜█▛             " ,
                            "    ▝               "
            };

    public static String getMark(boolean mark){
        return (mark) ? checkMark : xMark;
    }

    public static String getBox(boolean filled){
        return (filled) ? filledBox : emptyBox;
    }

    public static void bigCheckMark(Telemetry telemetry){
        for(int z = 0; z < bigCheckMark.length; z++){
            telemetry.addLine(bigCheckMark[z]);
        }
    }
}
