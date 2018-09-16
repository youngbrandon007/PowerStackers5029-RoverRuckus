package org.firstinspires.ftc.teamcode.PSRobotLibs.examples.driveExample;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.PSRobot;
import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.hardware.PSServo;

/**
 * Created by young on 8/7/2017.
 */
@TeleOp(name = "PineEx-ServoCS", group = "Linear Opmode")
@Disabled

public class ServoExample extends LinearOpMode {
    PSRobot robot;

    PSServo servo;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new PSRobot(this);
//        servo = robot.servoHandler.("D", 0.5);

        waitForStart();
            while(opModeIsActive()){
                if (gamepad1.a){
                    servo.setPosition(1);
                } else {
                    servo.setPosition(0.5);
                }
            }
//        robot.drive.encoderDrive(0.5, "4in",4);
//        robot.drive.setDirectPower(-1, -1);
//        Thread.sleep(1000);
    }
}
