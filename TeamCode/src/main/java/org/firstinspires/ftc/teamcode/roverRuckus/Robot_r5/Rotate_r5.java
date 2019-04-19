package org.firstinspires.ftc.teamcode.roverRuckus.Robot_r5;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "t.rot", group = "r5")
@Disabled
public class Rotate_r5 extends Config_r5 {


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
                .turn(Math.toRadians(180))
                .build();

        drive.followTrajectory(trajectory);
        time.reset();
    }

    @Override
    public void loop() {
        if (drive.isFollowingTrajectory()) {
            telemetry.addData("error",drive.getFollowingError());
            drive.update();
        } else {
            robot.drive.mecanum.updateMecanumThirdPerson(gamepad1, 1.0, Math.toRadians(gyro.getHeading() - cal));
        }
        estimatedPose = drive.getEstimatedPose();
        telemetry.addData("pose", estimatedPose);
        telemetry.addData("gyro", Math.toDegrees(estimatedPose.getHeading()));
//        telemetry.addData("drive.pos", drive.getWheelPositions());
        telemetry.update();
    }

}
