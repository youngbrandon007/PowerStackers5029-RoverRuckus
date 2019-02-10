package org.firstinspires.ftc.teamcode.roverRuckus.Robot_r4;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.followers.MecanumPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.PSRobotLibs.PSTelemetry;
import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.PSConfigOpMode;
import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.PSEnum;
import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.PSRobot;
import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.hardware.PSMotor;
import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.hardware.PSServo;
import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.vision.UVC.UVCCamera;
import org.firstinspires.ftc.teamcode.RobotLive.RobotLiveData;
import org.firstinspires.ftc.teamcode.RobotLive.RobotLiveSend;
import org.firstinspires.ftc.teamcode.roverRuckus.Robot_r4.DriveConstants_r4;
import org.firstinspires.ftc.teamcode.roverRuckus.Robot_r4.Paths.ConstantsLoader;
import org.jetbrains.annotations.NotNull;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

abstract class Config_r4 extends PSConfigOpMode {

    Drive drive;
    Camera camera;
    Collector collector;
    Lift lift;
    Gyro gyro;

    @Override
    public void config(OpMode opMode) {
        robot = new PSRobot(opMode);

        //robot parts
        drive = new Drive();
        collector = new Collector();
        lift = new Lift();
        gyro = new Gyro();

        //extras
        camera = new Camera();
    }


    class Drive extends MecanumDrive {
        final List<PSMotor> motors;
        PSMotor leftFront;
        PSMotor rightFront;
        PSMotor leftBack;
        PSMotor rightBack;

        PSServo markerDepositor;
        double plf;
        double plb;
        double prf;
        double prb;

        boolean thirdPerson = false;

        TrajectoryFollower trajectoryFollower;

        PIDCoefficients HEADING_PID;
        PIDCoefficients LATERAL_PID;
        public final double K = (17 + 17) / 4;

        double PIDoutput = 0.0;

        double P = 0.1;
        double I = 0.0;
        double D = 0.1;
        Vector2d estimatedPosition = new Vector2d(0,0);
        double[] lastRotations;

        public Drive() {
            super(DriveConstants_r4.TRACK_WIDTH);
            DriveConstants_r4.BASE_CONSTRAINTS = ConstantsLoader.getDriveConstraints();
            double[] rotationPID = ConstantsLoader.getRotationPIDVA();
            double[] motionPID = ConstantsLoader.getMotionPIDVA();
            HEADING_PID = new PIDCoefficients(rotationPID[0], rotationPID[1], rotationPID[2]);
            LATERAL_PID = new PIDCoefficients(motionPID[0], motionPID[1], motionPID[2]);
            DriveConstants_r4.kV = motionPID[3];
            DriveConstants_r4.kA = motionPID[4];

            leftFront = robot.motorHandler.newDriveMotor("D.LF", PSEnum.MotorLoc.LEFTFRONT, 20);
            rightFront = robot.motorHandler.newDriveMotor("D.RF", PSEnum.MotorLoc.RIGHTFRONT, 20);
            leftBack = robot.motorHandler.newDriveMotor("D.LB", PSEnum.MotorLoc.LEFTBACK, 20);
            rightBack = robot.motorHandler.newDriveMotor("D.RB", PSEnum.MotorLoc.RIGHTBACK, 20);
            motors = Arrays.asList(leftFront, leftBack, rightBack, rightFront);

            markerDepositor = robot.servoHandler.newServo("D.Marker",180,0.35,true);

            trajectoryFollower = new MecanumPIDVAFollower(this, LATERAL_PID, HEADING_PID,
                    DriveConstants_r4.kV, DriveConstants_r4.kA, DriveConstants_r4.kStatic);
            for (PSMotor motor : motors) {
                motor.motorObject.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
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

        @Deprecated
        public double distanceTraveled() {
            double[] pos = getPosition();
            double distance = Math.sqrt(Math.pow(pos[0], 2) + Math.pow(pos[1], 2));
            return distance;
        }


        public void setMecanum(double angle, double speed, boolean PID) {
            robot.drive.mecanum.setMecanum(angle, speed, (PID) ? PIDoutput : 0, 1.0);
        }

        public void releaseMarker(){
            markerDepositor.setPosition(1);
        }
        public void unreleaseMarker(){
            markerDepositor.setPosition(0.35);
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

        @Override
        public double getExternalHeading() {
            return Math.toRadians(-gyro.getHeading());
        }

        @NotNull
        @Override
        public List<Double> getWheelPositions() {
            List<Double> wheelPositions = new ArrayList<>();
            for (PSMotor motor : motors) {
                wheelPositions.add(DriveConstants_r4.encoderTicksToInches(motor.getEncoderPosition()));
            }
            return wheelPositions;
        }

        @Override
        public void setMotorPowers(double v, double v1, double v2, double v3) {
            leftFront.setPower(v3);
            leftBack.setPower(v2);
            rightBack.setPower(-v1);
            rightFront.setPower(-v);
            telemetry.addData("V1",v);
            telemetry.addData("V2",v1);
            telemetry.addData("V3",v2);
            telemetry.addData("V4",v3);
        }

        public TrajectoryBuilder trajectoryBuilder() {
            return new TrajectoryBuilder(getEstimatedPose(), DriveConstants_r4.BASE_CONSTRAINTS);
        }

        public void followTrajectory(Trajectory trajectory) {
            trajectoryFollower.followTrajectory(trajectory);
        }

        public void updateFollower() {
            trajectoryFollower.update(getEstimatedPose());
        }

        public void update() {
            getEstimatedPose();
            updateFollower();
        }

        public boolean isFollowingTrajectory() {
            return trajectoryFollower.isFollowing();
        }

        public Pose2d getFollowingError() {
            return trajectoryFollower.getLastError();
        }

        public Pose2d getEstimatedPose() {
            double[] rotations = new double[4];

            for (int i = 0; i < 4; i++) {
                int encoderPosition = motors.get(i).getEncoderPosition();
                rotations[i] = driveEncoderTicksToRadians(encoderPosition);

            }

            if (lastRotations != null) {
                double[] rotationDeltas = new double[4];
                for (int i = 0; i < 4; i++) {
                    rotationDeltas[i] = rotations[i] - lastRotations[i];
                }

                Vector2d robotPoseDelta = getPoseDelta(rotationDeltas).pos();
                Vector2d fieldPoseDelta = robotPoseDelta.rotated(Math.toRadians(gyro.getHeading()));



                estimatedPosition = estimatedPosition.plus(fieldPoseDelta);
            }
            lastRotations = rotations;
            return new Pose2d(new Vector2d(estimatedPosition.getX(), estimatedPosition.getY()), Math.toRadians(gyro.getHeading()));
//            return new Pose2d(new Vector2d(0, 0), 0);
        }


        double driveEncoderTicksToRadians(int ticks) {
            double ticksPerRev = 28*20;
            return 2 * Math.PI * ticks / ticksPerRev;
        }

        public Pose2d getPoseDelta(double[] rot) {
            if (rot.length != 4) {
                throw new IllegalArgumentException("length must be four");
            }
            double RADIUS = 2;
            double x = RADIUS * (rot[0] + rot[1] - rot[2] - rot[3]) / 4;
            double y = RADIUS * (rot[0] - rot[1] + rot[2] - rot[3]) / 4;
            double h = RADIUS * (-rot[0] - rot[1] - rot[2] - rot[3]) / (4 * K);
            return new Pose2d(x, y, h);
        }
    }

    class Collector {
        public PSMotor extension;
        public PSMotor shooterRight;
        public PSMotor shooterLeft;

        public PSServo collectorRotate;

        public Collector() {
            extension = robot.motorHandler.newMotor("C.E", 10);
            shooterLeft = robot.motorHandler.newMotor("C.L", 3.7);
            shooterRight = robot.motorHandler.newMotor("C.R", 3.7);
            collectorRotate = robot.servoHandler.newServo("C.rotate",100,0,false);
        }

        public void setCollectorPower(double power){
            shooterRight.setPower(-power);
        }

    }

    class Lift {
        class Bridge {
            public PSServo rotateR;
            public PSServo rotateL;
            public PSServo doorServo;
            public double[] right = new double[]{0.01, 0.09}; //first value 0 degrees in robot, second 180 degrees toward lander
            public double[] left = new double[]{0.75, 0.25};
            public final double init = -20;
            public final double vert = 90;
            public final double out = 210;

            public Bridge() {
                rotateL = robot.servoHandler.newServo("L.B.L", 240, .5, false);
                rotateR = robot.servoHandler.newServo("L.B.R", 240, .5, false);
                doorServo = robot.servoHandler.newServo("L.B.D", 140, .5, false);
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

        public PSServo ratchet;
        public final double dropInit = .4;
        public final double dropNormal = .7;
        public Bridge bridge;

        public Lift() {
            extension = robot.motorHandler.newMotor("L.E", 70);
            ratchet = robot.servoHandler.newServo("L.ratchet", 140, 0.2, false);
            bridge = new Bridge();

            extension.motorObject.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        public void ratchetOn() {
            lift.ratchet.setPosition(0.6);
        }

        public void ratchetOff() {
            ratchet.setPosition(0.8);
        }
    }

    class Gyro {
//                NavxMicroNavigationSensor navxMicro;
//        IntegratingGyroscope gyro;
        BNO055IMU gyro;
        Orientation angles;
        double cal = 0;


        public Gyro() {
//            navxMicro = hardwareMap.get(NavxMicroNavigationSensor.class,"navx");
//            gyro =(IntegratingGyroscope)navxMicro;
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            gyro = hardwareMap.get(BNO055IMU.class, "gyro");
            gyro.initialize(parameters);

        }

        public double getHeading() {
            angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            return (-angles.firstAngle) + cal;
        }

        public Orientation getOrientation() {
            angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            return angles;
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

    class Auto {

    }

    static class con {
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
