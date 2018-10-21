package org.firstinspires.ftc.teamcode.roverRuckus.Testing.subsystemTest;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.PSConfigOpMode;
import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.PSEnum;
import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.PSRobot;
import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.drive.control.motion.PIDController;
import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.hardware.PSMotor;

abstract class config extends PSConfigOpMode{

    Drive drive;

    @Override
    public void config(OpMode opMode) {
        robot = new PSRobot(opMode);
        drive = new Drive();
    }

    public void testMotor(){
        drive.motorRF.setPower(1.0);
    }

    class Drive{
        PSMotor motorRF;
        PSMotor motorLF;
        PSMotor motorRB;
        PSMotor motorLB;
        double angle;
        public PIDCoefficients SPEEDPID = new PIDCoefficients(-0.5, 0, 0);
        public PIDController pidController = new PIDController(SPEEDPID);

        public Drive(){
            motorRF = robot.motorHandler.newDriveMotor("RF", PSEnum.MotorLoc.RIGHTFRONT, 1);
            motorLF = robot.motorHandler.newDriveMotor("LF", PSEnum.MotorLoc.RIGHTFRONT, 1);
            motorRB = robot.motorHandler.newDriveMotor("RB", PSEnum.MotorLoc.RIGHTFRONT, 1);
            motorLB = robot.motorHandler.newDriveMotor("LB", PSEnum.MotorLoc.RIGHTFRONT, 1);
            pidController.setOutputBounds(-1,1);
            pidController.reset();
        }

        public void updateSpeed(double angle, double speed){
            pidController.setSetpoint(speed);

        }
        public void update(){
//            robot.drive.mecanum.setMecanum(angle,pidController.getError(getSpeed()));
        }

//        public double getSpeed() {
//
//        }
    }
}
