package org.firstinspires.ftc.teamcode.roverRuckus.Robot_r4;


import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.path.heading.LinearInterpolator;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.hardware.PSMotor;

import static java.lang.Math.abs;
import static java.lang.Math.atan2;
import static java.lang.Math.sqrt;

@TeleOp(name = "r4.Tele", group = "r4")
public class TeleOp_r4 extends Config_r4 {


    double cal = 0.0;
    private boolean shooterOn = false;

    private boolean pathDrive = false;
    Pose2d placePos = new Pose2d(50, 57, 225);
    Trajectory trajectory;

    @Override
    public void init() {
        config(this);
        lift.ratchetOn();
        telemetry.addData("gyro", "ready");
        telemetry.update();

        for (PSMotor motor : drive.motors) {
            motor.motorObject.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
    }

    @Override
    public void loop() {
        //Drive switch drive and rotation sticks
//        if (gamepad1.x) {
//            cal = gyro.getHeading();
//            drive.thirdPerson = true;
//        }else if(gamepad1.a){
//            drive.thirdPerson = false;
//        }
        telemetry.addData("gyro", gyro.getHeading());
        if(gamepad1.y){
            Pose2d pos = drive.getEstimatedPose();
            double heading = gyro.getHeading();
            placePos = new Pose2d(pos.getX(), pos.getY(), heading);
        }


        if (gamepad1.b) {
            if (pathDrive) {
                if(drive.isFollowingTrajectory()) {
                    drive.update();
                }
            } else {
                Pose2d pos = drive.getEstimatedPose();
                double heading = gyro.getHeading();
                trajectory = new TrajectoryBuilder(new Pose2d(pos.getX(), pos.getY(), heading), new DriveConstraints(40.0, 60.0, 2, 3))
                        .lineTo(new Vector2d(placePos.getX(), placePos.getY()))
                        .build();
                drive.followTrajectory(trajectory);
                pathDrive = true;
            }
        } else {
            pathDrive = false;
            if (drive.thirdPerson) {
                robot.drive.mecanum.updateMecanumThirdPerson(gamepad1, (gamepad1.right_stick_button) ? 1.0 : .5, -Math.toRadians(gyro.getHeading() - cal));
            } else {
                robot.drive.mecanum.updateMecanum(gamepad1, (gamepad1.right_stick_button) ? 1.0 : .5);
            }
        }



        //collector
        collector.extension.setPower((gamepad1.dpad_up||gamepad2.right_bumper) ? 1 : ((gamepad1.dpad_down||gamepad2.left_bumper)?-1:0));
        if (gamepad1.x||gamepad2.x) {
            shooterOn = true;
        }else if(gamepad1.a||gamepad2.y){
            shooterOn = false;
        }
        collector.setCollectorPower(gamepad1.right_trigger-(gamepad1.left_trigger));
        if (shooterOn){
            collector.shooterLeft.setPower(-1);
        } else{
            collector.shooterLeft.setPower(0);
        }


        if (gamepad1.right_bumper||gamepad2.a){
            collector.collectorRotate.setPosition(0.2);
        } else {
            collector.collectorRotate.setPosition(0.9);
        }
        //Lift
        lift.extension.setPower((gamepad2.dpad_up) ? 1.0 : (gamepad2.dpad_down) ? -1.0 : 0.0);
        if (!(abs(gamepad2.left_stick_x) < 0.6f && abs(gamepad2.left_stick_y) < 0.6f)) {
            if (gamepad2.left_stick_x < -.9) telemetry.addData("bridge ", lift.bridge.setBridge2(0));
            if (gamepad2.left_stick_x > .9) telemetry.addData("bridge ", lift.bridge.setBridge2(180));
//            telemetry.addData("bride.pos", Math.toDegrees(atan2(-gamepad2.left_stick_y, -gamepad2.left_stick_x)));
//
        }
        telemetry.addData("bridgeServoPos",lift.bridge.rotateL.getCachedPosition());

        if (gamepad2.dpad_right) {

            lift.bridge.rotateL.setPosition(lift.bridge.rotateL.getCachedPosition()+0.0005);
        } else if (gamepad2.dpad_left) {
            lift.bridge.rotateL.setPosition(lift.bridge.rotateL.getCachedPosition()-0.0005);
        }
        
        if (gamepad2.right_stick_button) {
            lift.ratchetOn();
        } else if (gamepad2.left_stick_button) {
            lift.ratchetOff();
        }

        lift.bridge.doorServo.setPosition((gamepad1.left_bumper) ? lift.dropNormal : lift.dropInit);

    }
}
