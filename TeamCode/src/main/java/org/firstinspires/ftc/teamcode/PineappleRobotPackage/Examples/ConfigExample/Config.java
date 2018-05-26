package org.firstinspires.ftc.teamcode.PineappleRobotPackage.Examples.ConfigExample;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PineappleEnum;
import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PineappleMotor;
import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PineappleConfigLinearOpMode;
import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PineappleRobot;

/**
 * Created by Brandon on 6/26/2017.
 */

abstract public class Config extends PineappleConfigLinearOpMode {

    public PineappleMotor testMotor;

    public void config(LinearOpMode linearOpMode){
        robotHandler = new PineappleRobot(linearOpMode);
        testMotor = robotHandler.motorHandler.newMotor("testMotor", PineappleEnum.MotorType.NEV40);
    }
}
