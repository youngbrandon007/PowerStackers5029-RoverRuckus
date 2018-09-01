package org.firstinspires.ftc.teamcode.PSRobotLibs.lib.drive.control.path;


import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.drive.control.Pose2d;
import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.drive.control.motion.MotionConstraints;
import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.drive.control.motion.MotionGoal;
import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.drive.control.motion.MotionProfile;
import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.drive.control.motion.MotionProfileGenerator;
import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.drive.control.motion.MotionState;
import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.drive.control.path.parametric.ParametricPath;

public class ParametricSegment implements TrajectorySegment {
    private ParametricPath path;
    private MotionProfile profile;
    private MotionConstraints constraints;

    public ParametricSegment(ParametricPath path, MotionConstraints constraints) {
        this.path = path;
        this.constraints = constraints;
        MotionState start = new MotionState(0, 0, 0, 0, 0);
        MotionGoal goal = new MotionGoal(path.length(), 0);
        profile = MotionProfileGenerator.generateProfile(start, goal, constraints);
    }

    @Override
    public double duration() {
        return profile.end().t;
    }

    @Override
    public Pose2d start() {
        return path.start();
    }

    @Override
    public Pose2d end() {
        return path.end();
    }

    @Override
    public Pose2d getPose(double time) {
        return path.getPose(profile.get(time).x);
    }

    @Override
    public Pose2d getVelocity(double time) {
        MotionState motionState = profile.get(time);
        return path.getDerivative(motionState.x).multiplied(motionState.v);
    }

    @Override
    public Pose2d getAcceleration(double time) {
        MotionState motionState = profile.get(time);
        return path.getSecondDerivative(motionState.x).multiplied(Math.pow(motionState.v, 2))
                .added(path.getDerivative(motionState.x).multiplied(motionState.a));
    }

    @Override
    public void stopPrematurely(double time) {
        profile = MotionProfileGenerator.generateStoppingProfile(profile.get(time), constraints);
    }

    public double timeAtPos(double position) {
        return profile.timeAtPos(position);
    }

    public ParametricPath path() {
        return path;
    }
}
