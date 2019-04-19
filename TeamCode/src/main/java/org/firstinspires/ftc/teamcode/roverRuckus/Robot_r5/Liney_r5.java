package org.firstinspires.ftc.teamcode.roverRuckus.Robot_r5;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "t.liney", group = "r5")
@Disabled
public class Liney_r5 extends Config_r5 {


    ElapsedTime time = new ElapsedTime();
    private double cal = 0;
    Trajectory trajectory;

    private Pose2d estimatedPose = new Pose2d(0, 0,0);

    @Override
    public void init() {
        config(this);

    }

    @Override
    public void init_loop() {

        telemetry.addData("gyro.heading", gyro.getHeading());
        telemetry.addData("P",drive.HEADING_PID.kP);
        telemetry.addData("I",drive.HEADING_PID.kI);
        telemetry.addData("D",drive.HEADING_PID.kD);
        telemetry.addData("P",drive.LATERAL_PID.kP);
        telemetry.addData("I",drive.LATERAL_PID.kI);
        telemetry.addData("D",drive.LATERAL_PID.kD);
        telemetry.update();
    }

    @Override
    public void start() {
        trajectory = drive.trajectoryBuilder()
                .lineTo(new Vector2d(0, -48), new ConstantInterpolator(0))
//                .strafeRight(20)
//                .splineTo(new Pose2d(20,20,Math.PI/2))
//                .splineTo(new Pose2d(40,20,-Math.PI/2))

                .build();


        drive.followTrajectory(trajectory);
        time.reset();
    }

    @Override
    public void loop() {
        if (drive.isFollowingTrajectory()) {

            drive.update();
        } else {
            robot.drive.mecanum.updateMecanum(gamepad1, 1.0);
        }
        estimatedPose = drive.getEstimatedPose();
        telemetry.addData("error",drive.getFollowingError());
        telemetry.addData("pose", estimatedPose);
        telemetry.addData("targetPose",trajectory.get(time.seconds()));
        telemetry.addData("gyro", Math.toDegrees(estimatedPose.getHeading()));

//        telemetry.addData("drive.pos", drive.getWheelPositions());
        telemetry.update();
    }

}
