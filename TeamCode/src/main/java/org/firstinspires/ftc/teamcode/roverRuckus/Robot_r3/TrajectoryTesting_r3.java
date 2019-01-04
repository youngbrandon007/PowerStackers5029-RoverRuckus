package org.firstinspires.ftc.teamcode.roverRuckus.Robot_r3;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.path.heading.HeadingInterpolator;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.hardware.PSMotor;

@TeleOp(name = "r3.Trajectory", group = "r3")
@Disabled
public class TrajectoryTesting_r3 extends Config_r3 {


    ElapsedTime time = new ElapsedTime();
    private double cal = 0;

    private Pose2d estimatedPose = new Pose2d(0, 0,0);

    @Override
    public void init() {
        config(this);

    }

    @Override
    public void init_loop() {

        telemetry.addData("gyro.heading", gyro.getHeading());
        telemetry.update();
    }

    @Override
    public void start() {
        Trajectory trajectory = drive.trajectoryBuilder()
                .lineTo(new Vector2d(24,0))
//                .splineTo(new Pose2d(20,20,Math.PI/2))
//                .splineTo(new Pose2d(40,20,-Math.PI/2))
                .build();

        drive.followTrajectory(trajectory);
        time.reset();
    }

    @Override
    public void loop() {
        if (drive.isFollowingTrajectory()) {
            estimatedPose = drive.getEstimatedPose();
            telemetry.addData("pose", estimatedPose);
            telemetry.addData("gyro", Math.toDegrees(estimatedPose.getHeading()));
            telemetry.addData("error", drive.getFollowingError());
            drive.update();
            telemetry.update();
        }

    }

}
