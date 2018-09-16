package org.firstinspires.ftc.teamcode.roverRuckus.other;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.PSEnum;
import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.PSRobot;
import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.hardware.PSMotor;
import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.hardware.PSServo;

public class basicProtoypingTele extends OpMode {
    PSRobot robot;
    PSMotor rightDrivetMotor;
    PSMotor leftDriveMotor;
    PSMotor otherMotor;
    PSServo servo;
    @Override
    public void init() {
        robot = new PSRobot(this);
        rightDrivetMotor = robot.motorHandler.newMotor("rdrive", 40);
        leftDriveMotor = robot.motorHandler.newMotor("ldrive", 40);
        otherMotor = robot.motorHandler.newMotor("motor", 40);
        servo = robot.servoHandler.newServo("servo", PSEnum.ServoTotalRotation.HS_485HB,0,false);
    }

    @Override
    public void loop() {

    }
}
