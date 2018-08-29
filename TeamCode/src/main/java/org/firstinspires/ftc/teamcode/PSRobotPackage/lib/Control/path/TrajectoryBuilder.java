package org.firstinspires.ftc.teamcode.PSRobotPackage.lib.Control.path;



import org.firstinspires.ftc.teamcode.PSRobotPackage.lib.Control.Angle;
import org.firstinspires.ftc.teamcode.PSRobotPackage.lib.Control.Pose2d;
import org.firstinspires.ftc.teamcode.PSRobotPackage.lib.Control.Vector2d;
import org.firstinspires.ftc.teamcode.PSRobotPackage.lib.Control.motion.MotionConstraints;
import org.firstinspires.ftc.teamcode.PSRobotPackage.lib.Control.path.parametric.CompositePath;
import org.firstinspires.ftc.teamcode.PSRobotPackage.lib.Control.path.parametric.LinePath;
import org.firstinspires.ftc.teamcode.PSRobotPackage.lib.Control.path.parametric.Marker;
import org.firstinspires.ftc.teamcode.PSRobotPackage.lib.Control.path.parametric.ParametricPath;
import org.firstinspires.ftc.teamcode.PSRobotPackage.lib.Control.path.parametric.SplinePath;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class TrajectoryBuilder {
    private Pose2d currentPose;
    private MotionConstraints axialConstraints, pointTurnConstraints;
    private List<TrajectorySegment> motionSegments;
    private List<ParametricPath> compositeSegments;
    private boolean composite;

    public TrajectoryBuilder(Pose2d pose, MotionConstraints axialConstraints, MotionConstraints pointTurnConstraints) {
        currentPose = pose;
        this.axialConstraints = axialConstraints;
        this.pointTurnConstraints = pointTurnConstraints;
        motionSegments = new ArrayList<>();
        compositeSegments = new ArrayList<>();
    }

    public TrajectoryBuilder lineToPose(Pose2d pose) {
        ParametricPath path = new LinePath(currentPose, pose);
        if (composite) {
            compositeSegments.add(path);
        } else {
            motionSegments.add(new ParametricSegment(path, axialConstraints));
        }
        currentPose = path.end();
        return this;
    }

    public TrajectoryBuilder lineTo(Vector2d pos) {
        return lineToPose(new Pose2d(pos, currentPose.heading()));
    }

    public TrajectoryBuilder turn(double angle) {
        return turnTo(Angle.norm(currentPose.heading() + angle));
    }

    public TrajectoryBuilder turnTo(double heading) {
        PointTurn pointTurn = new PointTurn(currentPose, heading, pointTurnConstraints);
        motionSegments.add(pointTurn);
        currentPose = new Pose2d(currentPose.pos(), heading);
        return this;
    }

    public TrajectoryBuilder forward(double distance) {
        return lineTo(currentPose.pos().added(new Vector2d(
                distance * Math.cos(currentPose.heading()),
                distance * Math.sin(currentPose.heading())
        )));
    }

    public TrajectoryBuilder back(double distance) {
        return forward(-distance);
    }

    public TrajectoryBuilder strafeLeft(double distance) {
        return lineTo(currentPose.pos().added(new Vector2d(
                distance * Math.cos(currentPose.heading() + Math.PI / 2),
                distance * Math.sin(currentPose.heading() + Math.PI / 2)
        )));
    }

    public TrajectoryBuilder strafeRight(double distance) {
        return strafeLeft(-distance);
    }

    public TrajectoryBuilder splineThrough(SplinePath.Type type, Pose2d... waypoints) {
        List<Pose2d> modifiedWaypoints = new ArrayList<>(Arrays.asList(waypoints));
        modifiedWaypoints.add(0, currentPose);
        CompositePath spline = CompositePath.fitSpline(type, modifiedWaypoints);
        if (composite) {
            compositeSegments.add(spline);
        } else {
            motionSegments.add(new ParametricSegment(spline, axialConstraints));
        }
        currentPose = spline.end();
        return this;
    }

    public TrajectoryBuilder splineThrough(Pose2d... waypoints) {
        return splineThrough(SplinePath.Type.CUBIC_HERMITIAN, waypoints);
    }

    public TrajectoryBuilder waitFor(double seconds) {
        motionSegments.add(new WaitSegment(currentPose, seconds));
        return this;
    }

    public TrajectoryBuilder addMarker(String name) {
        Marker marker = new Marker(name, currentPose);
        if (composite) {
            compositeSegments.add(marker);
        } else {
            motionSegments.add(new ParametricSegment(marker, axialConstraints));
        }
        return this;
    }

    public TrajectoryBuilder beginComposite() {
        if (composite) {
            closeComposite();
        }
        composite = true;
        return this;
    }

    public TrajectoryBuilder closeComposite() {
        composite = false;
        motionSegments.add(new ParametricSegment(new CompositePath(compositeSegments), axialConstraints));
        compositeSegments = new ArrayList<>();
        return this;
    }

    public Trajectory build() {
        if (composite) {
            closeComposite();
        }
        return new Trajectory(motionSegments);
    }
}
