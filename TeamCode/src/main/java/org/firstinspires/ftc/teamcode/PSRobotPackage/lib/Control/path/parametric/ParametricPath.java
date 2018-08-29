package org.firstinspires.ftc.teamcode.PSRobotPackage.lib.Control.path.parametric;

import org.firstinspires.ftc.teamcode.PSRobotPackage.lib.Control.Pose2d;



public interface ParametricPath {
    double length();
    Pose2d start();
    Pose2d end();
    Pose2d getPose(double displacement);
    Pose2d getDerivative(double displacement);
    Pose2d getSecondDerivative(double displacement);
}
