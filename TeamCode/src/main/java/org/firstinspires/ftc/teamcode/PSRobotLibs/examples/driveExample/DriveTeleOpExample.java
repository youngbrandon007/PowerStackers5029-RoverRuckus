package org.firstinspires.ftc.teamcode.PSRobotLibs.examples.driveExample;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.PSEnum;
import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.PSRobot;
import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.hardware.PSMotor;

/**
 * Created by Brandon on 7/14/2017.
 */

@TeleOp(name = "PineEx-Tele", group = "Linear Opmode")
@Disabled

public class DriveTeleOpExample extends LinearOpMode {
    PSRobot robot;

    PSMotor left;
    PSMotor right;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new PSRobot(this);

        left = robot.motorHandler.newDriveMotor("r", 1, true , true, PSEnum.MotorLoc.LEFT, 40);
        right = robot.motorHandler.newDriveMotor("l", 1, true , true, PSEnum.MotorLoc.RIGHT, 40);


        waitForStart();
        while (opModeIsActive()){

//            robot.drive.setPower(gamepad1.left_stick_y, gamepad1.right_stick_y);

        }
    }
}
