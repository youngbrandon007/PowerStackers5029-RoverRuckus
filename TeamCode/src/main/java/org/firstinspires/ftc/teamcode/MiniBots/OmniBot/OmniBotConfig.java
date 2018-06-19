package org.firstinspires.ftc.teamcode.MiniBots.OmniBot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PineappleConfigOpMode;
import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PineappleEnum;
import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PineappleMotor;
import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PineappleRobot;

abstract public class OmniBotConfig extends PineappleConfigOpMode{

    PineappleMotor leftFront;
    PineappleMotor rightFront;
    PineappleMotor leftBack;
    PineappleMotor rightBack;

    @Override
    public void config(OpMode opMode) {
        robotHandler = new PineappleRobot(opMode);

//        leftFront = robotHandler.motorHandler.newDriveMotor("LF", PineappleEnum.MotorLoc.LEFTFRONT);
//        rightFront = robotHandler.motorHandler.newDriveMotor("RF", PineappleEnum.MotorLoc.RIGHTFRONT);
//        leftBack = robotHandler.motorHandler.newDriveMotor("LB", PineappleEnum.MotorLoc.LEFTBACK);
//        rightBack = robotHandler.motorHandler.newDriveMotor("RB", PineappleEnum.MotorLoc.RIGHTBACK);

    }
}
