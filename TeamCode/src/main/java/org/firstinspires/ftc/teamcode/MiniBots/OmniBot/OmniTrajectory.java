package org.firstinspires.ftc.teamcode.miniBots.omniBot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.drive.control.Pose2d;
import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.drive.control.Vector2d;
import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.drive.control.motion.MotionConstraints;
import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.drive.control.motion.PIDFCoefficients;
import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.drive.control.path.Trajectory;
import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.drive.control.path.TrajectoryBuilder;
import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.drive.control.path.TrajectoryFollower;

import java.util.Arrays;
import java.util.Collections;

@Autonomous(name = "OmniTrajectory", group = "omnibot")
public class OmniTrajectory extends OmniBotConfig {
    //TODO values not tuned
    protected static MotionConstraints AXIAL_CONSTRAINTS = new MotionConstraints(30.0, 40.0, 160.0, MotionConstraints.EndBehavior.OVERSHOOT);
    protected static MotionConstraints POINT_TURN_CONSTRAINTS = new MotionConstraints(2.0, 2.67, 10.67, MotionConstraints.EndBehavior.OVERSHOOT);

    TrajectoryBuilder trajectoryBuilder = new TrajectoryBuilder(new Pose2d(0,0,Math.PI/2),AXIAL_CONSTRAINTS,POINT_TURN_CONSTRAINTS);
    Trajectory trajectory;
    private TrajectoryFollower trajectoryFollower;
    public static PIDFCoefficients HEADING_PIDF = new PIDFCoefficients(-0.5, 0, 0, 0.230, 0);
    public static PIDFCoefficients AXIAL_PIDF = new PIDFCoefficients(-0.05, 0, 0, 0.0177, 0);
    public static PIDFCoefficients LATERAL_PIDF = new PIDFCoefficients(-0.05, 0, 0, 0.0179, 0);
    private double[] powers;
    private Vector2d targetVel;
    private double targetOmega;
    Pose2d estimatedPose;
    public static final double K = (18 + 18) / 4;
    private double[] lastRotations;
    private Vector2d estimatedPosition;



    @Override
    public void init() {
        config(this);
        gyroSensor.calibrate();
        while (gyroSensor.isCalibrating()) {
            telemetry.addData("Gyro", "Calib");
            telemetry.update();
        }
        telemetry.addData("Gyro", "Done");
        telemetry.update();
        trajectory = trajectoryBuilder
                .splineThrough(new Pose2d(0,20,Math.PI/2),new Pose2d(10,40, Math.PI/4))
                .turnTo(-Math.PI/2)
                .lineToPose(new Pose2d(0,0,Math.PI/2))
                .build();


        trajectoryFollower = new TrajectoryFollower(HEADING_PIDF, AXIAL_PIDF, LATERAL_PIDF);

    }

    @Override
    public void start(){
        trajectoryFollower.follow(trajectory);
        telemetry.clearAll();
    }

    @Override
    public void loop() {
        estimatedPose = getEstimatedPose();
        Pose2d update =  trajectoryFollower.update(estimatedPose);
        targetVel = update.pos();
        targetOmega = update.heading();
        updatePowers();
        for (int i = 0; i < 4; i++) {
            psMotors[i].setPower(powers[i]);
        }

        telemetry.addData("X Position", estimatedPose.x());
        telemetry.addData("Y Position", estimatedPose.y());
        telemetry.addData("Heading", estimatedPose.heading());
    }

    private Pose2d getEstimatedPose() {
        double[] rotations = new double[4];

        for (int i = 0; i < 4; i++) {
            double encoderPosition = psMotors[i].getEncoderPosition();

        }
        if (lastRotations != null) {
            double[] rotationDeltas = new double[4];
            for (int i = 0; i < 4; i++) {
                rotationDeltas[i] = rotations[i] - lastRotations[i];
            }

            Vector2d robotPoseDelta = getPoseDelta(rotationDeltas).pos();
            Vector2d fieldPoseDelta = robotPoseDelta.rotated(Math.toRadians(gyroSensor.getHeading()));

            estimatedPosition = estimatedPosition.added(fieldPoseDelta);
        }
        lastRotations = rotations;
        return new Pose2d(estimatedPosition, Math.toRadians(gyroSensor.getHeading()));
    }

    private void updatePowers() {
        powers[0] = targetVel.x() - targetVel.y() - targetOmega;
        powers[1] = targetVel.x() + targetVel.y() - targetOmega;
        powers[2] = targetVel.x() - targetVel.y() + targetOmega;
        powers[3] = targetVel.x() + targetVel.y() + targetOmega;

        double max = Collections.max(Arrays.asList(1.0, Math.abs(powers[0]),
                Math.abs(powers[1]), Math.abs(powers[2]), Math.abs(powers[3])));

        for (int i = 0; i < 4; i++) {
            powers[i] /= max;
        }
    }

    public static Pose2d getPoseDelta(double[] rot) {
        if (rot.length != 4) {
            throw new IllegalArgumentException("length must be four");
        }
        double RADIUS = 2;
        double x = RADIUS * ( rot[0] + rot[1] + rot[2] + rot[3]) / 4;
        double y = RADIUS * (-rot[0] + rot[1] - rot[2] + rot[3]) / 4;
        double h = RADIUS * (-rot[0] - rot[1] + rot[2] + rot[3]) / (4 * K);
        return new Pose2d(x, y, h);
    }
}
