package org.firstinspires.ftc.teamcode.PineappleRobotPackage.Examples.DriveExample;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PSEnum;
import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PSMotor;
import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PSRobot;

/**
 * Created by Brandon on 6/26/2017.
 */

@TeleOp(name = "PineEx-Motor", group = "Linear Opmode")
@Disabled

public class MotorExample extends LinearOpMode {
    PSRobot robot;

    PSMotor testMotor;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new PSRobot(this);

        testMotor = robot.motorHandler.newMotor("test", 1, true , true, PSEnum.MotorType.NEV40);


        waitForStart();
        while (opModeIsActive()){
            testMotor.update(gamepad1.left_stick_x);


        }
    }
}
