package org.firstinspires.ftc.teamcode.PSRobotLibs.examples.driveExample;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.PSEnum;
import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.PSRobot;
import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.hardware.PSMotor;

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

        testMotor = robot.motorHandler.newMotor("test", 1, true , true, 40);


        waitForStart();
        while (opModeIsActive()){
            testMotor.update(gamepad1.left_stick_x);


        }
    }
}
