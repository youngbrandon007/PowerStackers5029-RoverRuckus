package org.firstinspires.ftc.teamcode.MiniBots.OmniBot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.GyroSensor;

import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.PSConfigOpMode;
import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.PSEnum;
import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.PSRobot;
import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.hardware.PSMotor;

abstract public class OmniBotConfig extends PSConfigOpMode {

    PSMotor leftFront;
    PSMotor rightFront;
    PSMotor leftBack;
    PSMotor rightBack;
    GyroSensor gyroSensor;
    PSMotor[] psMotors = {leftFront,rightFront,rightBack,leftBack};

    @Override
    public void config(OpMode opMode) {
        robotHandler = new PSRobot(opMode);
        gyroSensor = hardwareMap.gyroSensor.get("GS");
        leftFront = robotHandler.motorHandler.newDriveMotor("LF", PSEnum.MotorLoc.LEFTFRONT,40);
        rightFront = robotHandler.motorHandler.newDriveMotor("RF", PSEnum.MotorLoc.RIGHTFRONT,40);
        leftBack = robotHandler.motorHandler.newDriveMotor("LB", PSEnum.MotorLoc.LEFTBACK,40);
        rightBack = robotHandler.motorHandler.newDriveMotor("RB", PSEnum.MotorLoc.RIGHTBACK,40);
    }
}
