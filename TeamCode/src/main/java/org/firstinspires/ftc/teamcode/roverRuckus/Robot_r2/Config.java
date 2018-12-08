package org.firstinspires.ftc.teamcode.roverRuckus.Robot_r2;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.PSConfigOpMode;
import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.PSEnum;
import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.PSRobot;
import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.hardware.PSMotor;
import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.hardware.PSServo;
import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.utils.PSGeneralUtils;
import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.vision.UVC.UVCCamera;
import org.firstinspires.ftc.teamcode.RobotLive.RobotLiveData;
import org.firstinspires.ftc.teamcode.RobotLive.RobotLiveSend;

import java.util.ArrayList;

abstract class Config extends PSConfigOpMode {


    ///////////////////////////////////----Run Settings----///////////////////////////////////////////
    static class set {
        static boolean doLanding = false;
        static boolean doSample = true;
        static boolean useCamera = true;

        //Real Match these value must be FALSE
        static boolean saveImages = true;
        static boolean robotLiveEnabled = false;
    }
    ///////////////////////////////////////////////////////////////////////////////////////////////////

    Drive drive;
    RobotLive robotLive;
    PSTelemetry tel;
    Camera camera;
    Collector collector;
    Transfer transfer;
    Lift lift;
    Gyro gyro;

    @Override
    public void config(OpMode opMode) {
        robot = new PSRobot(opMode);

        //robot parts
        drive = new Drive();
        collector = new Collector();
        transfer = new Transfer();
        lift = new Lift();
        gyro = new Gyro();

        //extras
        tel = new PSTelemetry(telemetry);
        camera = new Camera();
    }


    class Drive {
        PSMotor leftFront;
        PSMotor rightFront;
        PSMotor leftBack;
        PSMotor rightBack;
        double plf;
        double plb;
        double prf;
        double prb;

        boolean thirdPerson = false;

        double PIDoutput = 0.0;

        double P = 0.1;
        double I = 0.0;
        double D = 0.1;

        public Drive() {
            leftFront = robot.motorHandler.newDriveMotor("D.LF", PSEnum.MotorLoc.LEFTFRONT, 40);
            rightFront = robot.motorHandler.newDriveMotor("D.RF", PSEnum.MotorLoc.RIGHTFRONT, 40);
            leftBack = robot.motorHandler.newDriveMotor("D.LB", PSEnum.MotorLoc.LEFTBACK, 40);
            rightBack = robot.motorHandler.newDriveMotor("D.RB", PSEnum.MotorLoc.RIGHTBACK, 40);
        }

        public void resetEncoders() {
            plf = drive.leftFront.getEncoderDistance(4);// - plf;
            plb = drive.leftBack.getEncoderDistance(4);// - plb;
            prf = drive.rightFront.getEncoderDistance(4);// - prf;
            prb = drive.rightBack.getEncoderDistance(4);// - prb;
        }

        public double[] getPosition() {
            double lf = drive.leftFront.getEncoderDistance(4) - plf;
            double lb = drive.leftBack.getEncoderDistance(4) - plb;
            double rf = drive.rightFront.getEncoderDistance(4) - prf;
            double rb = drive.rightBack.getEncoderDistance(4) - prb;
            double d1 = (lf - rb) / 2.0;
            double d2 = (lb - rf) / 2.0;
            return new double[]{d1, d2};
        }

        public double distanceTraveled() {
            double[] pos = getPosition();
            double distance = Math.sqrt(Math.pow(pos[0], 2) + Math.pow(pos[1], 2));
            return distance;
        }

        public void setMecanum(double angle, double speed, boolean PID) {
            robot.drive.mecanum.setMecanum(angle, speed, (PID) ? PIDoutput : 0, 1.0);
        }

        @Deprecated
        public boolean autoDrive(double angle, double distance, double speed, double PID) {
            angle -= 45;
            double xTarget = Math.sin(Math.toRadians(angle)) * distance;
            double yTarget = Math.cos(Math.toRadians(angle)) * distance;
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

    class Collector {
        public PSMotor extension;
        public PSMotor sweeper;
        public PSServo door;
        public PSServo ramp;
        public Boolean sweeperOn = false;

        public Collector() {
            extension = robot.motorHandler.newMotor("C.E", 10);
            sweeper = robot.motorHandler.newMotor("C.S", 3.7);
            door = robot.servoHandler.newServo("C.D", 197, 0.00, false);
            ramp = robot.servoHandler.newServo("C.R", 100, .62, true);

        }

        public void initDoor() {
            door.setPosition(.6);
        }

        public void openDoor() {
            door.setPosition(0.01);
        }

        public void closeDoor() {
            door.setPosition(0.55);
        }

        public void rampDown() {
            ramp.setPosition(0.72);
        }

        public void rampUp() {
            ramp.setPosition(0.62);
        }
    }

    class Transfer {
        public PSMotor shooter;
        public boolean shooterOn = false;
        public CRServo feeder;
        public boolean feederOn = false;

        public Transfer() {
            shooter = robot.motorHandler.newMotor("T.S", 3.7);
            feeder = hardwareMap.crservo.get("T.F");
        }
    }

    class Lift {
        class Bridge {
            public PSServo rotateR;
            public PSServo rotateL;
            public double[] right = new double[]{0.55, 0.05}; //first value 0 degrees in robot, second 180 degrees toward lander
            public double[] left = new double[]{0.75, 0.25};
            public final double init = -20;
            public final double vert = 90;
            public final double out = 210;

            public Bridge() {
                rotateL = robot.servoHandler.newServo("L.L", 240, .5, false);
                rotateR = robot.servoHandler.newServo("L.R", 240, .5, false);
            }

            public void setBridge(double input) {
                rotateR.setPosition(Math.abs(input));
                rotateL.setPosition(Math.abs(input));
            }

            public double setBridge2(double degrees) {
                degrees += (degrees < -90) ? 360 : (degrees > 270) ? -360 : 0;
                double r = Range.scale(degrees, 0, 180, right[0], right[1]);
                double l = Range.scale(degrees, 0, 180, left[0], left[1]);
                rotateR.setPosition(r);
                rotateL.setPosition(l);
                return r;

            }
        }

        public PSMotor extension;
        public PSServo drop;
        public PSServo ratchet;
        public final double dropInit = .5;
        public final double dropNormal = .2;
        public final double dropOpposite = .8;
        public double dropPos = .5;
        public Bridge bridge;

        public Lift() {
            extension = robot.motorHandler.newMotor("L.E", 70);
            drop = robot.servoHandler.newServo("L.SO", 140, .5, true);
            ratchet = robot.servoHandler.newServo("L.D", 140, 0.3, false);
            bridge = new Bridge();
        }

        public void ratchetOn() {
            lift.ratchet.setPosition(0.6);
        }

        public void ratchetOff() {
            ratchet.setPosition(0);
        }
    }

    class Gyro {
        //        NavxMicroNavigationSensor navxMicro;
//        IntegratingGyroscope gyro;
        BNO055IMU imu;
        Orientation angles;


        public Gyro() {
//            navxMicro = hardwareMap.get(NavxMicroNavigationSensor.class,"navx");
//            gyro =(IntegratingGyroscope)navxMicro;
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            imu = hardwareMap.get(BNO055IMU.class, "imu");
            imu.initialize(parameters);

        }
        public double getAngle() {
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            return angles.firstAngle;
        }
        public double getAngle1() {
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            return angles.firstAngle;
        }

        public double getAngle2() {
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            return angles.secondAngle;
        }

        public double getAngle3() {
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            return angles.thirdAngle;
        }
    }

    /*
     * Auto
     */
    class RobotLive {
        RobotLiveData data;
        String ip = "http://50.5.236.197:400";

        public RobotLive() {
            data = RobotLiveSend.createNewRun(ip);
        }

        public void send() {
            RobotLiveSend.send(data, ip);
        }
    }

    enum AutoTasks {
        UNRATCHET, LAND, TAKEPICTURE, SAMPLEPICTURE, SAMPLEDRIVE, WAITPICTURE, PARK
    }

    class Auto {

    }

    static class con {
        final static double[] sampleAngle = new double[]{0, 60, 90, 120};
        final static double[] sampleDis = new double[]{0, 12, 10, 12};

        final static double[] sampleRange = new double[]{-100, 100};
    }

    class Camera {
        UVCCamera camera;

        public void load(UVCCamera.Callback callback) {
            if (camera == null) {
                camera = UVCCamera.getCamera(callback);
            }
        }

        public void start() {
            if (camera != null) {
                camera.start();
            }
        }

        public void stop() {
            if (camera != null) {
                camera.stop();
                camera = null;
            }
        }
    }
}
