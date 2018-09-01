package org.firstinspires.ftc.teamcode.PSRobotLibs.lib.drive.control.path;



import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.drive.control.Angle;
import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.drive.control.Pose2d;
import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.drive.control.motion.MotionConstraints;
import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.drive.control.motion.MotionGoal;
import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.drive.control.motion.MotionProfile;
import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.drive.control.motion.MotionProfileGenerator;
import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.drive.control.motion.MotionState;

public class PointTurn implements TrajectorySegment {
    private Pose2d startPose;
    private double endHeading;
    private MotionProfile profile;

    public PointTurn(Pose2d startPose, double endHeading, MotionConstraints constraints) {
        this.startPose = startPose;
        this.endHeading = endHeading;
        double startHeading = startPose.heading(), displacement;
        displacement = Angle.norm(endHeading - startHeading);
        MotionState start = new MotionState(0, 0, 0, 0, 0);
        MotionGoal goal = new MotionGoal(displacement, 0);
        profile = MotionProfileGenerator.generateProfile(start, goal, constraints);
    }

    @Override
    public double duration() {
        return profile.end().t;
    }

    @Override
    public Pose2d start() {
        return startPose;
    }

    @Override
    public Pose2d end() {
        return new Pose2d(startPose.pos(), endHeading);
    }

    @Override
    public Pose2d getPose(double time) {
        return new Pose2d(startPose.pos(), Angle.norm(startPose.heading() + profile.get(time).x));
    }

    @Override
    public Pose2d getVelocity(double time) {
        return new Pose2d(0, 0, profile.get(time).v);
    }

    @Override
    public Pose2d getAcceleration(double time) {
        return new Pose2d(0, 0, profile.get(time).a);
    }

    @Override
    public void stopPrematurely(double time) {
        // do nothing; TODO: is there something better to do here?
    }
}
