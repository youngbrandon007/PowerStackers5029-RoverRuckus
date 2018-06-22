package org.firstinspires.ftc.teamcode.MiniBots.OmniBot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PSConfigOpMode;
import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PSMotor;
import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PSRobot;

abstract public class OmniBotConfig extends PSConfigOpMode {

    PSMotor leftFront;
    PSMotor rightFront;
    PSMotor leftBack;
    PSMotor rightBack;

    @Override
    public void config(OpMode opMode) {
        robotHandler = new PSRobot(opMode);

//        leftFront = robotHandler.motorHandler.newDriveMotor("LF", PineappleEnum.MotorLoc.LEFTFRONT);
//        rightFront = robotHandler.motorHandler.newDriveMotor("RF", PineappleEnum.MotorLoc.RIGHTFRONT);
//        leftBack = robotHandler.motorHandler.newDriveMotor("LB", PineappleEnum.MotorLoc.LEFTBACK);
//        rightBack = robotHandler.motorHandler.newDriveMotor("RB", PineappleEnum.MotorLoc.RIGHTBACK);

    }
}
