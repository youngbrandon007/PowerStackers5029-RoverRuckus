package org.firstinspires.ftc.teamcode.roverRuckus.Testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.PSConfigOpMode;
import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.PSEnum;
import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.PSRobot;
import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.hardware.PSMotor;

@TeleOp(name = "_prototype_", group = "proto")
public class prototypeTesting extends OpMode {

    PSMotor driveLeft;
    PSMotor driveRight;
    PSMotor extendLift;
    PSMotor turrentVert;

    PSRobot robot;

    @Override
    public void init() {
        robot = new PSRobot(this);
        driveLeft = robot.motorHandler.newDriveMotor("L", PSEnum.MotorLoc.LEFT, 40);
        driveRight = robot.motorHandler.newDriveMotor("R", PSEnum.MotorLoc.RIGHT, 40);

        extendLift = robot.motorHandler.newMotor("lift", 40);
        turrentVert = robot.motorHandler.newMotor("turr", 40);
    }

    @Override
    public void loop() {
        //dirve (one may need to e reversed)
        driveLeft.setPower(gamepad1.left_stick_y);
        driveRight.setPower(gamepad1.right_stick_y);


        //dpad up and down for one motor
        extendLift.setPower((gamepad1.dpad_up) ? .3 : (gamepad1.dpad_down) ? -.3 : 0.0);
        //a and y for one motor
        turrentVert.setPower((gamepad1.y) ? .3 : (gamepad1.a) ? -.3 : 0.0);
    }
}
