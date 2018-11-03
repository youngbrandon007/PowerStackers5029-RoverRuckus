package org.firstinspires.ftc.teamcode.roverRuckus.Robot_r2;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.PSConfigOpMode;
import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.PSEnum;
import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.PSRobot;
import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.hardware.PSMotor;

abstract class Config extends PSConfigOpMode {


    Drive drive;

    @Override
    public void config(OpMode opMode) {
        robot = new PSRobot(opMode);
        drive = new Drive();
    }


    class Drive{
        PSMotor leftFront;
        PSMotor rightFront;
        PSMotor leftBack;
        PSMotor rightBack;
        double plf;
        double plb;
        double prf;
        double prb;

        public Drive(){
            leftFront = robot.motorHandler.newDriveMotor("LF", PSEnum.MotorLoc.LEFTFRONT,40);
            rightFront = robot.motorHandler.newDriveMotor("RF", PSEnum.MotorLoc.RIGHTFRONT,40);
            leftBack = robot.motorHandler.newDriveMotor("LB", PSEnum.MotorLoc.LEFTBACK,40);
            rightBack = robot.motorHandler.newDriveMotor("RB", PSEnum.MotorLoc.RIGHTBACK,40);
        }

        public void resetEncoders(){
            plf = drive.leftFront.getEncoderPosition();// - plf;
            plb = drive.leftBack.getEncoderPosition();// - plb;
            prf = drive.rightFront.getEncoderPosition();// - prf;
            prb = drive.rightBack.getEncoderPosition();// - prb;
        }

        public double[] getPosition(){
            double lf = drive.leftFront.getEncoderPosition()- plf;
            double lb = drive.leftBack.getEncoderPosition()- plb;
            double rf = drive.rightFront.getEncoderPosition()- prf;
            double rb = drive.rightBack.getEncoderPosition()- prb;
            double d1 = (lf-rb)/2.0;
            double d2 = (lb-rf)/2.0;
            return new double[]{d1, d2};
        }

        public boolean autoDive(double angle, double distance, double speed, double PID){
            angle -= 45;
            double xTarget = Math.sin(Math.toRadians(angle))*distance;
            double yTarget = Math.cos(Math.toRadians(angle))*distance;
            double[] currentPos = getPosition();
            double xPos = currentPos[1];
            double yPos = currentPos[0];

            double travelAngle = Math.toDegrees(Math.atan2(xTarget - xPos, yTarget - yPos));
            double distanceToPoint = Math.sqrt(Math.pow(xTarget - xPos, 2) + Math.pow(yTarget - yPos, 2));


            telemetry.addData("pos.x", xPos);
            telemetry.addData("pos.y", yPos);
            telemetry.addData("target.x", xTarget);
            telemetry.addData("target.y", yTarget);
            telemetry.addData("distance", distanceToPoint);
            telemetry.addData("angle", travelAngle);

            robot.drive.mecanum.setMecanum(Math.toRadians(travelAngle), speed, PID, 1.0);

            return (distanceToPoint < 50);
        }
    }
}
