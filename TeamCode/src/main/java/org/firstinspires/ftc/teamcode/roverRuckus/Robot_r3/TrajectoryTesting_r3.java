package org.firstinspires.ftc.teamcode.roverRuckus.Robot_r3;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.drive.control.Pose2d;
import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.drive.control.Vector2d;
import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.drive.control.motion.MotionConstraints;
import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.drive.control.motion.PIDFCoefficients;
import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.drive.control.path.Trajectory;
import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.drive.control.path.TrajectoryBuilder;
import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.drive.control.path.TrajectoryFollower;
import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.hardware.PSMotor;

import java.util.Arrays;
import java.util.Collections;

@TeleOp(name = "r3.Trajectory", group = "r3")
public class TrajectoryTesting_r3 extends Config_r3 {

    protected static MotionConstraints AXIAL_CONSTRAINTS = new MotionConstraints(30.0, 40.0, 160.0, MotionConstraints.EndBehavior.OVERSHOOT);
    protected static MotionConstraints POINT_TURN_CONSTRAINTS = new MotionConstraints(2.0, 2.67, 10.67, MotionConstraints.EndBehavior.OVERSHOOT);

    TrajectoryBuilder trajectoryBuilder = new TrajectoryBuilder(new Pose2d(0, 0, Math.PI / 2), AXIAL_CONSTRAINTS, POINT_TURN_CONSTRAINTS);
    Trajectory trajectory;
    private TrajectoryFollower trajectoryFollower;
    public static PIDFCoefficients HEADING_PIDF = new PIDFCoefficients(-0.5, 0, 0, 0.230, 0);
    public static PIDFCoefficients AXIAL_PIDF = new PIDFCoefficients(-0.05, 0, 0, 0.0177, 0);
    public static PIDFCoefficients LATERAL_PIDF = new PIDFCoefficients(-0.05, 0, 0, 0.0179, 0);
    private double[] powers;
    private Vector2d targetVel;
    private double targetOmega;
    public static final double K = (18 + 18) / 4;
    private double[] lastRotations;
    private Vector2d estimatedPosition = new Vector2d(0, 0);
    ElapsedTime time = new ElapsedTime();
    private double cal = 0;
    PSMotor[] psMotors = new PSMotor[4];

    @Override
    public void init() {
        config(this);
        psMotors[0] = drive.leftFront;
        psMotors[1] = drive.leftBack;
        psMotors[2] = drive.rightBack;
        psMotors[3] = drive.rightFront;


//        while (gyro.navxMicro.isCalibrating())  {
//            telemetry.addData("gyro.status", "calibrating");
//        }
        telemetry.addData("gyro.status", "ready");
    }

    @Override
    public void init_loop() {

        telemetry.addData("gyro.heading", gyro.getHeading());
        telemetry.update();
    }

    @Override
    public void start() {
        time.reset();
    }

    @Override
    public void loop() {
        if (gamepad1.left_stick_button) {
            cal = gyro.getHeading();
        }
        telemetry.addData("pose.x", getEstimatedPose().x());
        telemetry.addData("pose.y", getEstimatedPose().y());
        telemetry.addData("gyro", gyro.getHeading());
        robot.drive.mecanum.updateMecanumThirdPerson(gamepad1, (gamepad1.right_stick_button) ? 1.0 : .5, Math.toRadians(gyro.getHeading() - cal));
        telemetry.update();
    }

    private Pose2d getEstimatedPose() {
        double[] rotations = new double[4];

        for (int i = 0; i < 4; i++) {
            int encoderPosition = psMotors[i].getEncoderPosition();
            rotations[i] = driveEncoderTicksToRadians(encoderPosition);
            if (i == 2 || i == 3) {
                rotations[i] = -rotations[i];
            }
        }

        if (lastRotations != null) {
            double[] rotationDeltas = new double[4];
            for (int i = 0; i < 4; i++) {
                rotationDeltas[i] = rotations[i] - lastRotations[i];
            }

            Vector2d robotPoseDelta = getPoseDelta(rotationDeltas).pos();
            Vector2d fieldPoseDelta = robotPoseDelta.rotated(Math.toRadians(gyro.getHeading()));

            estimatedPosition = estimatedPosition.added(fieldPoseDelta);
        }
        lastRotations = rotations;
        return new Pose2d(estimatedPosition, Math.toRadians(gyro.getHeading()));
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

    private double driveEncoderTicksToRadians(int ticks) {
        double ticksPerRev = psMotors[0].cpr;
        return 2 * Math.PI * ticks / ticksPerRev;
    }

    public static Pose2d getPoseDelta(double[] rot) {
        if (rot.length != 4) {
            throw new IllegalArgumentException("length must be four");
        }
        double RADIUS = 2;
        double x = RADIUS * (rot[0] + rot[1] + rot[2] + rot[3]) / 4;
        double y = RADIUS * (-rot[0] + rot[1] - rot[2] + rot[3]) / 4;
        double h = RADIUS * (-rot[0] - rot[1] + rot[2] + rot[3]) / (4 * K);
        return new Pose2d(x, y, h);
    }
}
