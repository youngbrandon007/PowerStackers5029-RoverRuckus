package org.firstinspires.ftc.teamcode.roverRuckus.Testing.cameraTesting;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.PSConfigOpMode;
import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.PSRobot;
import org.firstinspires.ftc.teamcode.RobotLiveTest.RobotLiveDataTemp;



abstract class config extends PSConfigOpMode{

    RobotLiveDataTemp data;

    @Override
    public void config(OpMode opMode) {
        robot = new PSRobot(opMode);
    }
}
