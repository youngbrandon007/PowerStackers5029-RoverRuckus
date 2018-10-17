package org.firstinspires.ftc.teamcode.roverRuckus.Testing.subsystemTest;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.PSConfigOpMode;
import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.hardware.PSMotor;
import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.hardware.PSServo;

abstract class config extends PSConfigOpMode{
    @Override
    public void config(OpMode opMode) {

    }

    class drive{
        PSMotor motorRF;
        PSMotor motorLF;
        PSMotor motorRB;
        PSMotor motorLB;
    }
}
