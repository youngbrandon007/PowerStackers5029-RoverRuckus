package org.firstinspires.ftc.teamcode.roverRuckus.Robot_r4;

import android.os.Trace;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.path.heading.LinearInterpolator;
import com.acmerobotics.roadrunner.path.heading.SplineInterpolator;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.roverRuckus.Robot_r4.Config_r4;
import org.firstinspires.ftc.teamcode.roverRuckus.Robot_r4.Transition.AutoTransitioner;

@Autonomous(name="TransitionTest", group = "r4")
public class Auto_test extends Config_r4 {

    Trajectory trajectory;

    enum Action{
        DRIVETOPICK, PICK, DRIVETOPLACE
    }

    Action action = Action.PICK;

    @Override
    public void init() {
        config(this);

        AutoTransitioner.transitionOnStop(this, "r4.Tele");

        drive.estimatedPosition = new Vector2d(48, 48);
        gyro.cal = 225;
    }

    @Override
    public void loop() {
        Pose2d est = drive.getEstimatedPose();
        telemetry.addData("pos.x", est.getX());
        telemetry.addData("pos.y", est.getY());
        telemetry.addData("pos.head",  Math.toDegrees(est.getHeading()));
        Pose2d min = getPickLocation(new Vector2d(24,24));
        telemetry.addData("go.x", min.getX());
        telemetry.addData("go.y", min.getY());
        telemetry.addData("go.head", Math.toDegrees(min.getHeading()));
        switch(action){
            case DRIVETOPICK:
                if (drive.isFollowingTrajectory()) {
                    drive.update();
                }else{
                    action = Action.PICK;
                }
                break;
            case PICK:
                robot.drive.mecanum.updateMecanum(gamepad1, 1.0);
                if(gamepad1.x){
                    drive.estimatedPosition = new Vector2d(48, 48);
                    gyro.cal = 225;
                }
                if(gamepad1.b){
                    trajectory = new TrajectoryBuilder(drive.getEstimatedPose(), new DriveConstraints(50.0, 70.0, 2, 3))
                           // .lineTo(new Vector2d(0, 0),LinearInterpolator(drive.getEstimatedPose().getHeading(), 0))
                            .lineTo(new Vector2d(48,48), new SplineInterpolator(drive.getEstimatedPose().getHeading(), Math.toRadians(225)))
                            .build();

                    drive.followTrajectory(trajectory);

                    action = Action.DRIVETOPLACE;
                }
                if(gamepad1.y){
                    Pose2d go = getPickLocation(new Vector2d(24,24));

                    trajectory = new TrajectoryBuilder(drive.getEstimatedPose(), new DriveConstraints(50.0, 70.0, 2, 3))
                            // .lineTo(new Vector2d(0, 0),LinearInterpolator(drive.getEstimatedPose().getHeading(), 0))
                            .lineTo(new Vector2d(go.getX(),go.getY()), new SplineInterpolator(drive.getEstimatedPose().getHeading(), go.getHeading()))
                            .build();

                    drive.followTrajectory(trajectory);

                    action = Action.DRIVETOPICK;
                }
                break;
            case DRIVETOPLACE:


                if (drive.isFollowingTrajectory()) {
                    drive.update();
                }else{
                    action = Action.PICK;
                }
                break;
        }

        telemetry.addData("Action", action);
    }

    public Pose2d getPickLocation(Vector2d mineralLoc){
        Pose2d robotPos = drive.getEstimatedPose();

        double rx = robotPos.getX();
        double ry = robotPos.getY();
        double mx = mineralLoc.getX();
        double my = mineralLoc.getY();

        double angle = Math.atan2(my - ry, mx - rx);
        angle += (angle < 0) ? 2* Math.PI : 0;

        if((rx - mx) == 0){
            //y=-5/6x+71.667
            //x=rx
            double i1x = rx;
            double i1y = (- (5.0/6.0)) * i1x + (71 + (2.0/3.0));

            //y=60
            //x=rx
            double i2x = rx;
            double i2y = 60;

            if(i1y >= 60){
                return new Pose2d(i2x, i2y, angle);
            }else{
                return new Pose2d(i1x, i1y, angle);
            }
        }else{
            double m = (ry - my) / (rx - mx);
            double b = ry - (m * rx);

            //y=-5/6x+71.667
            //y=mx+b
            //mx+b=-5/6x+71.667
            //x(m+5/6)=71.667-b
            //x=(71.667-b)/(m+5/6)

            double i1x = ((71 + (2.0/3.0)) - b) / (m + (5.0/6.0));
            double i1y = m*i1x + b;

            //y=60
            //y=mx+b
            //60=mx+b
            //(60-b)/m=x
            double i2x = (60 - b) / m;
            double i2y = 60;

            //x=64.4
            //y=mx+b
            //y=m*64.4+b
            double i3x = 64.4;
            double i3y = m*i3x + b;

            if(i1x >=  64.4){
                return new Pose2d(i3x, i3y, angle);
            }else if(i1y >= 60){
                return new Pose2d(i2x, i2y, angle);
            }else{
                return new Pose2d(i1x, i1y, angle);
            }
        }
    }
}
