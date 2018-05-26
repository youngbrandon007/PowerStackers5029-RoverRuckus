package org.firstinspires.ftc.teamcode.PineappleRobotPackage.Examples.DriveExample;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PineappleEnum;
import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PineappleMotor;
import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PineappleRobot;

/**
 * Created by Brandon on 6/26/2017.
 */

@TeleOp(name = "PineEx-Motor", group = "Linear Opmode")
@Disabled

public class MotorExample extends LinearOpMode{
    PineappleRobot robot;

    PineappleMotor testMotor;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new PineappleRobot(this);

        testMotor = robot.motorHandler.newMotor("test", 1, true , true, PineappleEnum.MotorType.NEV40);


        waitForStart();
        while (opModeIsActive()){
            testMotor.update(gamepad1.left_stick_x);


        }
    }
}
