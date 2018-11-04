package org.firstinspires.ftc.teamcode.roverRuckus.Robot_r2;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.PSConfigOpMode;
import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.PSEnum;
import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.PSRobot;
import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.hardware.PSMotor;
import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.utils.PSGeneralUtils;
import org.firstinspires.ftc.teamcode.RobotLive.RobotLiveData;
import org.firstinspires.ftc.teamcode.RobotLive.RobotLiveSend;

import java.util.ArrayList;

abstract class Config extends PSConfigOpMode {


    Drive drive;
    RobotLive robotLive;
    PSTelemetry tel;

    @Override
    public void config(OpMode opMode) {
        robot = new PSRobot(opMode);
        drive = new Drive();
        tel = new PSTelemetry(telemetry);
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

        double PIDoutput = 0.0;

        public Drive(){
            leftFront = robot.motorHandler.newDriveMotor("LF", PSEnum.MotorLoc.LEFTFRONT,40);
            rightFront = robot.motorHandler.newDriveMotor("RF", PSEnum.MotorLoc.RIGHTFRONT,40);
            leftBack = robot.motorHandler.newDriveMotor("LB", PSEnum.MotorLoc.LEFTBACK,40);
            rightBack = robot.motorHandler.newDriveMotor("RB", PSEnum.MotorLoc.RIGHTBACK,40);
        }

        public void resetEncoders(){
            plf = drive.leftFront.getEncoderDistance(4);// - plf;
            plb = drive.leftBack.getEncoderDistance(4);// - plb;
            prf = drive.rightFront.getEncoderDistance(4);// - prf;
            prb = drive.rightBack.getEncoderDistance(4);// - prb;
        }

        public double[] getPosition(){
            double lf = drive.leftFront.getEncoderDistance(4)- plf;
            double lb = drive.leftBack.getEncoderDistance(4)- plb;
            double rf = drive.rightFront.getEncoderDistance(4)- prf;
            double rb = drive.rightBack.getEncoderDistance(4)- prb;
            double d1 = (lf-rb)/2.0;
            double d2 = (lb-rf)/2.0;
            return new double[]{d1, d2};
        }

        public double distanceTraveled(){
            double[] pos = getPosition();
            double distance = Math.sqrt(Math.pow(pos[0], 2) + Math.pow(pos[1], 2));
            return distance;
        }

        public void setMecanum(double angle, double speed, boolean PID){
            robot.drive.mecanum.setMecanum(angle, speed, (PID) ? PIDoutput : 0, 1.0);
        }

        @Deprecated
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

    /*
     * Auto
     */
    class RobotLive{
        RobotLiveData data;
        String ip = "http://50.5.236.197:400";

        public RobotLive(){
            data = RobotLiveSend.createNewRun(ip);
        }

        public void send(){

        }
    }

    enum AutoTasks {
        LAND, SAMPLEPICTURE, SAMPLEDRIVE, SAMPLEDRIVEBACK, DRIVEROTATETOWALL, DRIVETOWALL, DRIVEROTATETODEPOT, DRIVETODEPOT, SAMPLESECONDDRIVE, SAMPLESECONDDRIVEBACK, CLAIM, PARKDRIVEBACK
    }

    class Auto{

    }

    static class con {
        final static double[] smapleAngle = new double[]{0, 75, 90, 105};
        final static double[] sampleDis = new double[]{0, 12, 10, 12};
    }
}
