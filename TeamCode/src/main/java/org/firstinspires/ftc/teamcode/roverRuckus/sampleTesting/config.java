package org.firstinspires.ftc.teamcode.roverRuckus.sampleTesting;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.PSConfigOpMode;
import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.PSRobot;
import org.firstinspires.ftc.teamcode.RobotLiveTest.RobotLiveDataTemp;

import RobotLiveDataSending.RobotLiveData;

abstract class config extends PSConfigOpMode{

    RobotLiveDataTemp data;

    @Override
    public void config(OpMode opMode) {
        robot = new PSRobot(opMode);
    }
}
