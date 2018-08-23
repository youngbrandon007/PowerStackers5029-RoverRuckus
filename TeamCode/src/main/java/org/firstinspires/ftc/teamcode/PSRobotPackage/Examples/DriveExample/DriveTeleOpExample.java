package org.firstinspires.ftc.teamcode.PSRobotPackage.Examples.DriveExample;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.PSRobotPackage.lib.PSEnum;
import org.firstinspires.ftc.teamcode.PSRobotPackage.lib.PSMotor;
import org.firstinspires.ftc.teamcode.PSRobotPackage.lib.PSRobot;

/**
 * Created by Brandon on 7/14/2017.
 */

@TeleOp(name = "PineEx-WorldTele", group = "Linear Opmode")
@Disabled

public class DriveTeleOpExample extends LinearOpMode {
    PSRobot robot;

    PSMotor left;
    PSMotor right;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new PSRobot(this);

        left = robot.motorHandler.newDriveMotor("r", 1, true , true, PSEnum.MotorLoc.LEFT, PSEnum.MotorType.NEV40);
        right = robot.motorHandler.newDriveMotor("l", 1, true , true, PSEnum.MotorLoc.RIGHT, PSEnum.MotorType.NEV40);


        waitForStart();
        while (opModeIsActive()){

//            robot.drive.setPower(gamepad1.left_stick_y, gamepad1.right_stick_y);

        }
    }
}
