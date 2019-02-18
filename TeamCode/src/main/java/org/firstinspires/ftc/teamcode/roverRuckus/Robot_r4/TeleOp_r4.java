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

    //calibaration value for 3rd person driving
    double cal = 0.0;
    //Toggle shooter variable
    private boolean shooterOn = false;

    //Automatic drive on/off
    private boolean pathDrive = false;
    //location to automatically drive robot to place
    Pose2d placePos = new Pose2d(50, 57, 225);
    // object for automatic drive
    Trajectory trajectory;

    private int automaticHang = 0;

    //init
    @Override
    public void init() {
        //initialize robot
        config(this);
        //set servo ratchet position to on
        lift.ratchetOn();
        //debug telemetry
        telemetry.addData("gyro", "ready");
        telemetry.update();

        //set drive type to driver prefrence
        for (PSMotor motor : drive.motors) {
            motor.motorObject.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
    }

    //loop
    @Override
    public void loop() {
        //thrid person driving, disable if driver prefers

        if (gamepad1.right_stick_button) {
            cal = gyro.getHeading();
            drive.thirdPerson = true;
        } else if (gamepad1.left_stick_button) {
            drive.thirdPerson = false;
        }
        //debug telemtry
        telemetry.addData("gyro", gyro.getHeading());

        //Set automatic drive position
        if (gamepad1.y) {
            placePos = drive.getEstimatedPose();
        }

        //run automatic drive
        if (gamepad1.b) {
            //if already started just update
            if (pathDrive) {
                //only if trajectory is still active update
                if (drive.isFollowingTrajectory()) {
                    drive.update();
                }
            } else {
                //create path to mineral place point
                trajectory = new TrajectoryBuilder(drive.getEstimatedPose(), new DriveConstraints(40.0, 60.0, 2, 3))
                        .lineTo(new Vector2d(placePos.getX(), placePos.getY()))
                        .build();
                //start follow trajectory
                drive.followTrajectory(trajectory);
                //set flag variable
                pathDrive = true;
            }
            //if not automatic drive then use joysticks
        } else {
            //set path to off
            pathDrive = false;
            //check if third person driving is on
            if (drive.thirdPerson) {
                //update drive using gyro angle
                robot.drive.mecanum.updateMecanumThirdPerson(gamepad1, (gamepad1.right_stick_button) ? 1.0 : .5, -Math.toRadians(gyro.getHeading() - cal));
            } else {
                //udpate drive without third person driving
                robot.drive.mecanum.updateMecanum(gamepad1, (gamepad1.right_stick_button) ? 1.0 : .5);
            }
        }

        //collector in/out
        collector.extension.setPower((gamepad1.dpad_up || gamepad2.right_bumper) ? 1 : ((gamepad1.dpad_down || gamepad2.left_bumper) ? -1 : 0));
        //shooter toggle
        if (gamepad1.x || gamepad2.x) {
            shooterOn = true;
        } else if (gamepad1.a || gamepad2.y) {
            shooterOn = false;
        }
        //shooter output
        if (shooterOn) {
            collector.shooterLeft.setPower(-1);
        } else {
            collector.shooterLeft.setPower(0);
        }
        //collectory power
        collector.setCollectorPower(gamepad1.right_trigger - (gamepad1.left_trigger));

        //lift/lower collectory
        if (gamepad1.right_bumper || gamepad2.a) {
            collector.collectorRotate.setPosition(0.2);
        } else {
            collector.collectorRotate.setPosition(0.9);
        }
        //Lift
        int liftEnc = lift.extension.getEncoderPosition();
        telemetry.addData("lift", liftEnc);
        if (gamepad2.right_stick_y > .8) {
            lift.extension.setPower((liftEnc > lift.liftBelowHookHeight + 40) ? -1.0 : (liftEnc < lift.liftBelowHookHeight - 40) ? 1.0 : 0.0);
        } else if (gamepad2.right_stick_y < -.8) {
            if(automaticHang == 0) {
                lift.extension.setPower((liftEnc > lift.liftHookHeight + 40) ? -1.0 : (liftEnc < lift.liftHookHeight - 40) ? 1.0 : 0.0);
                if(liftEnc < lift.liftHookHeight + 40 && liftEnc > lift.liftHookHeight - 40){
                    automaticHang = 1;
                }
            }else if(automaticHang == 1){
                lift.extension.setPower((liftEnc > lift.liftHangHeight + 40) ? -1.0 : (liftEnc < lift.liftHangHeight - 40) ? 1.0 : 0.0);
                if(liftEnc < lift.liftHangHeight + 40 && liftEnc > lift.liftHangHeight - 40){
                    lift.ratchetOff();
                }
            }
        } else {
            automaticHang = 0;
            lift.extension.setPower((gamepad2.dpad_up) ? -1.0 : (gamepad2.dpad_down) ? 1.0 : 0.0);
        }
        //set brdige servo position
        if (!(abs(gamepad2.left_stick_x) < 0.6f && abs(gamepad2.left_stick_y) < 0.6f)) {
//            if (gamepad2.left_stick_x < -.9) telemetry.addData("bridge ", lift.bridge.setBridge2(0));
//            if (gamepad2.left_stick_x > .9) telemetry.addData("bridge ", lift.bridge.setBridge2(180));

            telemetry.addData("bridge.pos", lift.bridge.setBridge2(Math.toDegrees(atan2(-gamepad2.left_stick_y, -gamepad2.left_stick_x))));
            telemetry.addData("bridge.angle", Math.toDegrees(atan2(-gamepad2.left_stick_y, -gamepad2.left_stick_x)));
//
        }
        telemetry.addData("bridgeServoPos", lift.bridge.bridgeRotate.getCachedPosition());

        //move bridge for finer adjustement
        if (gamepad2.dpad_right) {

            lift.bridge.bridgeRotate.setPosition(lift.bridge.bridgeRotate.getCachedPosition() + 0.001);
        } else if (gamepad2.dpad_left) {
            lift.bridge.bridgeRotate.setPosition(lift.bridge.bridgeRotate.getCachedPosition() - 0.001);
        }
        if (gamepad2.b) {
            lift.bridge.bridgeRotate.off();
        }

        //ratchet on/off
        if (gamepad2.right_stick_button) {
            lift.ratchetOn();
        } else if (gamepad2.left_stick_button) {
            lift.ratchetOff();
        }

        //mineral drop door position
        lift.bridge.doorServo.setPosition((gamepad1.left_bumper) ? lift.dropNormal : lift.dropInit);
    }
}
