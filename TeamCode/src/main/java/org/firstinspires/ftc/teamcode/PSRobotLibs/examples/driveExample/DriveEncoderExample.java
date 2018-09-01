package org.firstinspires.ftc.teamcode.PSRobotLibs.examples.driveExample;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.PSEnum;
import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.hardware.PSMotor;
import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.PSRobot;
import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.hardware.sensors.PSTouchSensor;

/**
 * Created by young on 8/7/2017.
 */
@TeleOp(name = "PineEx-DriveEncoder", group = "Linear Opmode")
@Disabled

public class DriveEncoderExample extends LinearOpMode {
    PSRobot robot;

    PSMotor left;
    PSMotor right;
    PSTouchSensor touch;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new PSRobot(this);

        left = robot.motorHandler.newDriveMotor("left", 1, true , true, PSEnum.MotorLoc.LEFT, PSEnum.MotorType.NEV40);
        right = robot.motorHandler.newDriveMotor("right", 1, true , true, PSEnum.MotorLoc.RIGHT, PSEnum.MotorType.NEV40);

        touch = robot.sensorHandler.newTouchSensor("touch");

        right.motorObject.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

//        robot.drive.encoderDrive(0.5, "4in",4);
//        robot.drive.setDirectPower(-1, -1);
//        Thread.sleep(1000);
    }
}
