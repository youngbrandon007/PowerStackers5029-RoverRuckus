//package org.firstinspires.ftc.teamcode.miniBots.humanoid;
//
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//
//import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.PSConfigOpMode;
//import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.PSEnum;
//import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.PSRobot;
//import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.hardware.PSMotor;
//import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.hardware.PSServo;
//
//public abstract class humanoidRobot{
//    PSMotor driveLeftMotor;
//    PSMotor driveRightMotor;
//    PSMotor armLeftMotor;
//    PSMotor armRightMotor;
//
//    PSServo headServo;
//
//    public void config(OpMode opMode) {
//        robotHandler = new PSRobot(opMode);
//        driveLeftMotor = robotHandler.motorHandler.newDriveMotor("l", PSEnum.MotorLoc.NONE, 60);
//        driveRightMotor = robotHandler.motorHandler.newDriveMotor("r", PSEnum.MotorLoc.NONE, 60);
//        armLeftMotor = robotHandler.motorHandler.newDriveMotor("la", PSEnum.MotorLoc.NONE, 60);
//        armRightMotor = robotHandler.motorHandler.newDriveMotor("ra", PSEnum.MotorLoc.NONE, 60);
//        headServo = robotHandler.servoHandler.newServo("hs", 360, 0.5, false);
//    }
//
//
//}
