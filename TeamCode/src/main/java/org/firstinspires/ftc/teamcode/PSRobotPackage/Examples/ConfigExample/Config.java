package org.firstinspires.ftc.teamcode.PSRobotPackage.Examples.ConfigExample;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.PSRobotPackage.lib.PSConfigLinearOpMode;
import org.firstinspires.ftc.teamcode.PSRobotPackage.lib.PSEnum;
import org.firstinspires.ftc.teamcode.PSRobotPackage.lib.PSMotor;
import org.firstinspires.ftc.teamcode.PSRobotPackage.lib.PSRobot;

/**
 * Created by Brandon on 6/26/2017.
 */

abstract public class Config extends PSConfigLinearOpMode {

    public PSMotor testMotor;

    public void config(LinearOpMode linearOpMode){
        robotHandler = new PSRobot(linearOpMode);
        testMotor = robotHandler.motorHandler.newMotor("testMotor", PSEnum.MotorType.NEV40);
    }
}
