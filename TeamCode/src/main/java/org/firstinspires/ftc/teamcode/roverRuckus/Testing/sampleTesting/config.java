package org.firstinspires.ftc.teamcode.roverRuckus.Testing.sampleTesting;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.PSConfigOpMode;
import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.PSEnum;
import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.PSRobot;
import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.hardware.PSMotor;
import org.firstinspires.ftc.teamcode.RobotLive.RobotLiveData;

abstract class config extends PSConfigOpMode{

    RobotLiveData data;
    PSMotor leftFront;
    PSMotor rightFront;
    PSMotor leftBack;
    PSMotor rightBack;

    @Override
    public void config(OpMode opMode) {
        robot = new PSRobot(opMode);
        leftFront = robot.motorHandler.newDriveMotor("LF", PSEnum.MotorLoc.LEFTFRONT,40);
        rightFront = robot.motorHandler.newDriveMotor("RF", PSEnum.MotorLoc.RIGHTFRONT,40);
        leftBack = robot.motorHandler.newDriveMotor("LB", PSEnum.MotorLoc.LEFTBACK,40);
        rightBack = robot.motorHandler.newDriveMotor("RB", PSEnum.MotorLoc.RIGHTBACK,40);

    }
}
