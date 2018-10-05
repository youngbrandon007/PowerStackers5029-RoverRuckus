package org.firstinspires.ftc.teamcode.miniBots.humanoid;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.PSEnum;
import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.PSRobot;
import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.hardware.PSMotor;
import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.hardware.PSServo;

@TeleOp(name = "human", group = "hi")
public class humanoidTele extends OpMode{

    PSRobot robotHandler;

    PSMotor driveLeftMotor;
    PSMotor driveRightMotor;
    PSMotor armLeftMotor;
    PSMotor armRightMotor;

    PSServo headServo;


    @Override
    public void init() {
        robotHandler = new PSRobot(this);
        driveLeftMotor = robotHandler.motorHandler.newDriveMotor("l", PSEnum.MotorLoc.NONE, 60);
        driveRightMotor = robotHandler.motorHandler.newDriveMotor("r", PSEnum.MotorLoc.NONE, 60);
        armLeftMotor = robotHandler.motorHandler.newDriveMotor("la", PSEnum.MotorLoc.NONE, 60);
        armRightMotor = robotHandler.motorHandler.newDriveMotor("ra", PSEnum.MotorLoc.NONE, 60);
        headServo = robotHandler.servoHandler.newServo("hs", 360, 0.5, false);


    }

    @Override
    public void loop() {
        driveRightMotor.setPower(-gamepad1.right_stick_y*.8);
        driveLeftMotor.setPower(gamepad1.left_stick_y*.8);

        armRightMotor.setPower((gamepad1.right_bumper) ? .1: -gamepad1.right_trigger*.7);
        armLeftMotor.setPower((gamepad1.left_bumper) ? -.1: gamepad1.left_trigger*.7);

        headServo.setPosition((gamepad1.dpad_right) ? .7 : (gamepad1.dpad_left) ?  .3 : .5);
    }
}
