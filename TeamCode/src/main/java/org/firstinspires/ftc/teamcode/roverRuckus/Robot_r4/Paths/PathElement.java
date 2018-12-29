package org.firstinspires.ftc.teamcode.roverRuckus.Robot_r4.Paths;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.acmerobotics.roadrunner.path.heading.HeadingInterpolator;

public class PathElement {

    public int type;
    public double x;
    public double y;
    public HeadingInterpolator head;
    public double rot;

    public PathElement(int type, double x, double y, double rot , HeadingInterpolator head){
        this.type = type;
        this.x = x;
        this.y = y;
        this.head = head;
        this.rot = rot;
    }

    //Statics
    public static PathElement newStart(double x, double y, double head){
        return new PathElement(PathPoints.START, x, y, head, null);
    }

    public static PathElement newSpline(double x, double y, double head, double rot){
        return new PathElement(PathPoints.SPLINEINTERPOLER, x, y, head, new ConstantInterpolator(Math.toRadians(rot)));
    }

    public static PathElement newSpline(double x, double y, double head){
        return new PathElement(PathPoints.SPLINE, x, y, head, null);
    }

    public static PathElement newStrafe(double x, double y){
        return new PathElement(PathPoints.STRAFE, x, y, 0.0, null);
    }

    public static PathElement newRot(double rot){
        return new PathElement(PathPoints.ROTATE, 0, 0, rot, null);
    }

    public static PathElement newDelay(double time){
        return new PathElement(PathPoints.DELAY, time, 0, 0, null);
    }

    public static PathElement newReverse(boolean reversed){
        return new PathElement(PathPoints.REVERSE, (reversed) ? 1 : 0, 0, 0, null);
    }

    public static PathElement newDrive(double distance){
        return new PathElement(PathPoints.DRIVE, distance, 0, 0, null);
    }

    public static PathElement[] newDelayFull(double time){
        return new PathElement[]{new PathElement(PathPoints.DELAY, time, 0, 0, null)};
    }

    //functions
    public Pose2d getPose2d(){
        return new Pose2d(x, y, getRot());
    }

    public Vector2d getVector2d(){
        return new Vector2d(x, y);
    }

    public HeadingInterpolator getInterpolar(){
        return head;
    }

    public double getRot(){
        return Math.toRadians(rot);
    }

    public double getDelay(){
        return (type == PathPoints.DELAY) ? x : 0.0;
    }

    public double distance(){
        return x;
    }

    public boolean reverse(){
        return (x == 1) ? true : false;
    }
}
