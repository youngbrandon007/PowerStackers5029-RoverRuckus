package org.firstinspires.ftc.teamcode.PSRobotLibs.examples.configExample;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.PSConfigLinearOpMode;
import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.PSEnum;
import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.PSRobot;
import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.hardware.PSMotor;

/**
 * Created by Brandon on 6/26/2017.
 */

abstract public class Config extends PSConfigLinearOpMode {

    public PSMotor testMotor;

    public void config(LinearOpMode linearOpMode){
        robotHandler = new PSRobot(linearOpMode);
        testMotor = robotHandler.motorHandler.newMotor("testMotor", 40);
    }
}
