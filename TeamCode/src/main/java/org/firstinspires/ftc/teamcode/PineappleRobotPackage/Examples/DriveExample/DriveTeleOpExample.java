package org.firstinspires.ftc.teamcode.PineappleRobotPackage.Examples.DriveExample;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PineappleEnum;
import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PineappleMotor;
import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PineappleRobot;

/**
 * Created by Brandon on 7/14/2017.
 */

@TeleOp(name = "PineEx-Tele", group = "Linear Opmode")
@Disabled

public class DriveTeleOpExample extends LinearOpMode {
    PineappleRobot robot;

    PineappleMotor left;
    PineappleMotor right;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new PineappleRobot(this);

        left = robot.motorHandler.newDriveMotor("r", 1, true , true, PineappleEnum.MotorLoc.LEFT, PineappleEnum.MotorType.NEV40);
        right = robot.motorHandler.newDriveMotor("l", 1, true , true, PineappleEnum.MotorLoc.RIGHT, PineappleEnum.MotorType.NEV40);


        waitForStart();
        while (opModeIsActive()){

//            robot.drive.setPower(gamepad1.left_stick_y, gamepad1.right_stick_y);

        }
    }
}
