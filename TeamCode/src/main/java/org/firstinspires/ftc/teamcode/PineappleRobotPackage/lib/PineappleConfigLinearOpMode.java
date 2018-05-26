package org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by Brandon on 6/26/2017.
 */

abstract public class PineappleConfigLinearOpMode extends LinearOpMode{
    public org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PineappleRobot robotHandler;

    abstract public void config(LinearOpMode linearOpMode);
}
