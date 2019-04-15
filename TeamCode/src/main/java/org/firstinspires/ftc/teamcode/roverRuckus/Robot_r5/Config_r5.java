package org.firstinspires.ftc.teamcode.roverRuckus.Robot_r5;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.drive.Localizer;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.followers.MecanumPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
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
import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.vision.UVC.UVCCamera;
import org.firstinspires.ftc.teamcode.RobotLive.RobotLiveData;
import org.firstinspires.ftc.teamcode.RobotLive.RobotLiveSend;
import org.firstinspires.ftc.teamcode.Paths.ConstantsLoader;
import org.jetbrains.annotations.NotNull;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

abstract class Config_r5 extends PSConfigOpMode {

    //Organization tree
    Drive drive; //Drive section
    Camera camera; //Camera object
    Collector collector; //collector seciton
    Lift lift; // lift section
    Gyro gyro; // gyro object


    @Override
    public void config(OpMode opMode) { //Create robot
        //create robot object using PSrobot
        robot = new PSRobot(opMode);

        // create and init all parts of robot
        drive = new Drive();
        collector = new Collector();
        lift = new Lift();
        gyro = new Gyro();

        // create other objects, not inited untill need to save battery
        camera = new Camera();
    }

    //drive object w/ drive functions and trajectory functions
    class Drive extends MecanumDrive {
        //motor objects
        final List<PSMotor> motors;
        PSMotor leftFront;
        PSMotor rightFront;
        PSMotor leftBack;
        PSMotor rightBack;
        //other drivebase motors and servos
        PSServo markerDepositor;

        //third person drive setting for teleOp
        boolean thirdPerson = false;

        //Trajectory object filled in later when trajectory is created
        TrajectoryFollower trajectoryFollower;

        //PID coefficient
        PIDCoefficients HEADING_PID;
        PIDCoefficients LATERAL_PID;
        //K coeffecient, (width + height) / 4
        public final double K = (17 + 17) / 4;

        //svaed PIDoutput for output to motors and telementry
        double PIDoutput = 0.0;

        //last estimated position used to find current position
        Vector2d estimatedPosition = new Vector2d(0,0);
        //encoder positions last update to find change
        double[] lastRotations;

        double xPrev = 0;
        double yPrev = 0;
        double lastAngle = 0;

        //init
        public Drive() {
            //load super class
            super(DriveConstants_r5.TRACK_WIDTH);
            // load drive constraints from settings app
            DriveConstants_r5.BASE_CONSTRAINTS = ConstantsLoader.getDriveConstraints();
            // load pid coeffecients
            double[] rotationPID = ConstantsLoader.getRotationPIDVA(); //heading
            double[] motionPID = ConstantsLoader.getMotionPIDVA(); //motion (lateral)
            //convert those into object for trajectory
            HEADING_PID = new PIDCoefficients(rotationPID[0], rotationPID[1], rotationPID[2]);
            LATERAL_PID = new PIDCoefficients(motionPID[0], motionPID[1], motionPID[2]);
            //save V and A coeffecients for feed forward
            DriveConstants_r5.kV = motionPID[3];
            DriveConstants_r5.kA = motionPID[4];
            //hardware map
            leftFront = robot.motorHandler.newDriveMotor("D.LF", PSEnum.MotorLoc.LEFTFRONT, 20);
            rightFront = robot.motorHandler.newDriveMotor("D.RF", PSEnum.MotorLoc.RIGHTFRONT, 20);
            leftBack = robot.motorHandler.newDriveMotor("D.LB", PSEnum.MotorLoc.LEFTBACK, 20);
            rightBack = robot.motorHandler.newDriveMotor("D.RB", PSEnum.MotorLoc.RIGHTBACK, 20);
            //add motors to list for easier handling of motors
            motors = Arrays.asList(leftFront, leftBack, rightBack, rightFront);

            //init other drive base servos and motors
            markerDepositor = robot.servoHandler.newServo("D.Marker",180,0.35,true);

            //init trajectory foller with both pid and other constants, trajectory added later in auto
            trajectoryFollower = new MecanumPIDVAFollower(this, LATERAL_PID, HEADING_PID,
                    DriveConstants_r5.kV, DriveConstants_r5.kA, DriveConstants_r5.kStatic);
            //set motor zero power behavoirs, in teleOp this is COAST (for easy control), brake in auto for faster stopping
            for (PSMotor motor : motors) {
                motor.motorObject.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
        }

        //Marker positions
        public void releaseMarker(){
            markerDepositor.setPosition(1);
        }
        public void unreleaseMarker(){
            markerDepositor.setPosition(0.32);
        }


        //trajectory callbacks
        // trajectory requesting gyro heading
        @Override
        public double getExternalHeading() {
            return Math.toRadians(-gyro.getHeading());
        }

        // trajectory request encoder values
        @NotNull
        @Override
        public List<Double> getWheelPositions() {
            List<Double> wheelPositions = new ArrayList<>();
            for (PSMotor motor : motors) {
                wheelPositions.add(DriveConstants_r5.encoderTicksToInches(motor.getEncoderPosition()));
            }
            return wheelPositions;
        }

        //trajectory output to motors
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

        //create starter of trajecotrybuilder to create trajecotry
        public TrajectoryBuilder trajectoryBuilder() {
            return new TrajectoryBuilder(getEstimatedPose(), DriveConstants_r5.BASE_CONSTRAINTS);
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

        //trajectory access
        public boolean isFollowingTrajectory() {
            return trajectoryFollower.isFollowing();
        }
        public Pose2d getFollowingError() {
            return trajectoryFollower.getLastError();
        }

        //calculate estimated position using change in encoder values and gro value
        public Pose2d getEstimatedPose() {
//            double[] rotations = new double[4];
//
//            for (int i = 0; i < 4; i++) {
//                int encoderPosition = motors.get(i).getEncoderPosition();
//                rotations[i] = driveEncoderTicksToRadians(encoderPosition);
//
//            }
//
//            if (lastRotations != null) {
//                double[] rotationDeltas = new double[4];
//                for (int i = 0; i < 4; i++) {
//                    rotationDeltas[i] = rotations[i] - lastRotations[i];
//                }
//
//                Vector2d robotPoseDelta = getPoseDelta(rotationDeltas).pos();
//                Vector2d fieldPoseDelta = robotPoseDelta.rotated(Math.toRadians(gyro.getHeading()));
//
//
//
//                estimatedPosition = estimatedPosition.plus(fieldPoseDelta);
//            }
//            lastRotations = rotations;
//            return new Pose2d(new Vector2d(estimatedPosition.getX(), estimatedPosition.getY()), Math.toRadians(gyro.getHeading()));

            double yTrackerWheelCounts = collector.shooterRight.getEncoderPosition();
            double xTrackerWheelCounts = collector.shooterLeft.getEncoderPosition();
            double yTrackerWheelCountsChange = yTrackerWheelCounts - yPrev;
            double xTrackerWheelCountsChange = xTrackerWheelCounts - xPrev;
            double wheelCirm = 3*Math.PI;
            double yWheelFromCenter = 7.125;
            double xWheelFromCenter = 7.375;
            double heading = gyro.getHeading();
            double angleChange = heading-lastAngle;
            double xTrackerWheelInches = -((wheelCirm*(xTrackerWheelCountsChange/4096.0))+(xWheelFromCenter*4.0*Math.PI*(angleChange/360.0)));
            double yTrackerWheelInches = -((wheelCirm*(yTrackerWheelCountsChange/4096.0))-(yWheelFromCenter*4.0*Math.PI*(angleChange/360.0)));
            xPrev = xTrackerWheelCounts;
            yPrev = yTrackerWheelCounts;
            Vector2d change = new Vector2d(xTrackerWheelInches, yTrackerWheelInches);
            change = change.rotated(Math.toRadians(heading));
            estimatedPosition = estimatedPosition.plus(change);
            lastAngle = heading;
            return new Pose2d(new Vector2d(estimatedPosition.getX(), estimatedPosition.getY()), Math.toRadians(heading));
        }
        public Pose2d getTrackerWheelPos(){
            int xTrackerWheelCounts = collector.shooterRight.getEncoderPosition();
            int yTrackerWheelCounts = collector.shooterLeft.getEncoderPosition();
            double wheelCirm = 3*Math.PI;
            double xWheelFromCenter = 7.125;
            double yWheelFromCenter = 7.375;
            double xTrackerWheelInches = (wheelCirm/xTrackerWheelCounts)-(xWheelFromCenter*2*Math.PI*(gyro.getAngleChange()/360));
            double yTrackerWheelInches = wheelCirm/yTrackerWheelCounts-(yWheelFromCenter*2*Math.PI*(gyro.getAngleChange()/360));

            return new Pose2d(xTrackerWheelInches,yTrackerWheelInches,gyro.getHeading());
        }
        //converting function
        double driveEncoderTicksToRadians(int ticks) {
            double ticksPerRev = 28*20;
            return 2 * Math.PI * ticks / ticksPerRev;
        }

        //calculate change in pos
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

    //collector object
    class Collector {
        //motors
        public PSMotor extension;
        public PSMotor shooterRight;
        public PSMotor shooterLeft;

        //servos
        public PSServo collectorRotate;

        //init
        public Collector() {
            extension = robot.motorHandler.newMotor("C.E", 10);
            shooterLeft = robot.motorHandler.newMotor("C.L", 3.7);
            shooterRight = robot.motorHandler.newMotor("C.R", 3.7);
            collectorRotate = robot.servoHandler.newServo("C.rotate",100,0,false);
            shooterLeft.motorObject.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        //set power, reversed
        public void setCollectorPower(double power){
            shooterRight.setPower(-power);
        }

    }

    //lift section object
    class Lift {
        //bridge part of lift
        class Bridge {
            //servo objects
            //public PSServo bridgeRotate;
            public CRServo bridgeLeft;
            public CRServo bridgeRight;
            public PSServo doorServo;
            public PSServo canopy;

            //servo scalling
            public double[] right = new double[]{0.04, 0.11}; //first value 0 degrees in robot, second 180 degrees toward lander

            //init
            public Bridge() {
                //hardware map
                //bridgeRotate = robot.servoHandler.newServo("L.B.R", 240, .5, false);
                doorServo = robot.servoHandler.newServo("L.B.D", 140, .5, false);
                bridgeLeft = hardwareMap.get(CRServo.class, "L.B.L");
                bridgeRight = hardwareMap.get(CRServo.class, "L.B.R");
                canopy = robot.servoHandler.newServo("L.B.C", 180, 0, false);
            }

            public void openBridge(){
                bridgeRight.setPower(1);
                bridgeLeft.setPower(-1);
            }

            public void closeBridge(){
                bridgeRight.setPower(-1);
                bridgeLeft.setPower(1);
            }

            public void stopBridge(){
                bridgeRight.setPower(0);
                bridgeLeft.setPower(0);
            }

            //old bridge method
//            @Deprecated
//            public void setBridge(double input) {
//                bridgeRotate.setPosition(Math.abs(input));
//            }
//
//            //new bridge with degrees input
//            public double setBridge2(double degrees) {
//                //set range to -90 to 270
//                degrees += (degrees < -90) ? 360 : (degrees > 270) ? -360 : 0;
//                //scale range to two motor settings
//                double r = Range.scale(degrees, 0, 180, right[0], right[1]);
////                double l = Range.scale(degrees, 0, 180, left[0], left[1]);
//                //output to motors
//                bridgeRotate.setPosition(r);
//                //return value for telemtry
//                return r;
//
//            }
        }

        //motors objects for lift
        public PSMotor extension;
        public int liftExtendTickCount = -8700;
        public int liftBelowHookHeight = 3000;
        public int liftHookHeight = 1000;
        public int liftHangHeight = 5500;

        //servo and positions
        public PSServo ratchet;
        public final double dropInit = .4;
        public final double dropNormal = .7;

        //brdige seciton object
        public Bridge bridge;

        //init
        public Lift() {
            //hardwaremap
            extension = robot.motorHandler.newMotor("L.E", 70);
            ratchet = robot.servoHandler.newServo("L.ratchet", 140, 0.2, false);
            bridge = new Bridge();

            //set settings of lift motor
            extension.motorObject.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        //rachet servo movement
        public void ratchetOn() {
            lift.ratchet.setPosition(0.4);
        }
        public void ratchetOff() {
            ratchet.setPosition(0.8);
        }
    }

    //gyro class, rev or navX
    class Gyro {
        //navX gyro
//         NavxMicroNavigationSensor navxMicro;
//        IntegratingGyroscope gyro;
        //Rev Gyro
        BNO055IMU gyro;
        Orientation angles;
        //Calibration value for auto
        double cal = 0;
        double lastGyroAngle =0 ;
        double angleChange =0;
        //init
        public Gyro() {
            //init navX
//            navxMicro = hardwareMap.get(NavxMicroNavigationSensor.class,"navx");
//            gyro =(IntegratingGyroscope)navxMicro;
            //init Rev
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            gyro = hardwareMap.get(BNO055IMU.class, "gyro");
            gyro.initialize(parameters);
//            lastGyroAngle = getHeading();
//            angleChange = getHeading()-lastGyroAngle;
        }

        //get heading of gyro
        public double getHeading() {
            angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            return (-angles.firstAngle) + cal;
        }
        public double getAngleChange(){
            angleChange = getHeading()-lastGyroAngle;
            lastGyroAngle = getHeading();
            return angleChange;
        }
        //get all angles
        public Orientation getOrientation() {
            angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            return angles;
        }

    }

    /*
     * Auto
     */
    //robotlive object for output to dashboard
    //not used at comp
    //for debugging
    class RobotLive {
        // data stroage object
        RobotLiveData data;
        //web ip
        String ip = "http://50.5.236.197:400";

        //init
        public RobotLive() {
            //create data storage object
            data = RobotLiveSend.createNewRun(ip);
        }

        //send data that has been added
        public void send() {
            RobotLiveSend.send(data, ip);
        }
    }

    //camera object only used in auto to save battery
    class Camera {
        //custom camera object
        UVCCamera camera;

        //load camera (init)
        public void load(UVCCamera.Callback callback) {
            //check if camera already exists
            if (camera == null) {
                // find camera that is plugged into usb hub
                camera = UVCCamera.getCamera(callback);
            }
        }

        //start camera stream
        public void start() {
            // check if camera exists
            if (camera != null) {
                //start camera stream
                camera.start();
            }
        }

        //stop camera stream
        public void stop() {
            //check if camera exists
            if (camera != null) {
                //stop
                camera.stop();
                //remove storage of camera
                camera = null;
            }
        }
    }
}
