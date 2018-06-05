package org.firstinspires.ftc.teamcode.RelicRecoveryWorlds;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;


/**
 * Created by Brandon on 1/8/2018.
 */
@TeleOp(name = "TeleOp")

public class WorldTele extends WorldConfig {

    //        boolean bClicked = false;
    double collectorSpeed = 0;
    boolean autoLiftOn = false;
    ElapsedTime collectorRPMTimer = new ElapsedTime();
    ElapsedTime collectorRPM = new ElapsedTime();
    // double prervPos = motorCollectLeft.getEncoderPosition();
    //double prervTime = collectorRPMTimer.milliseconds();
    //double liftTarget = 0;
    boolean thirdPersonOn = false;
    boolean rightArm = false;
    boolean leftArm = false;
    double thirdPersonCal = 0.0;
    double speed = 0;


    @Override
    public void init() {
        config(this);
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void loop() {
        //            double encoderDif = motorCollectLeft.getEncoderPosition() - prervPos;
//            double timeDif = collectorRPMTimer.milliseconds() - prervTime;
//            double RPM = Math.abs((60000 / timeDif) * encoderDif);
//            telemetry.addData("Encoder Val Right", motorCollectRight.getEncoderPosition());
//            telemetry.addData("Encoder Val Left", motorCollectLeft.getEncoderPosition());Ad
//            telemetry.addData("Collector RPM", RPM);
        //PID
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double gyro = angles.firstAngle;

        if (gamepad1.right_bumper) {
            thirdPersonOn = true;
            thirdPersonCal = gyro;
        } else if (gamepad1.left_bumper) {
            thirdPersonOn = false;
        }
        if (gamepad1.a) {
            rightArm = false;
            leftArm = false;
        } else if (gamepad1.b) {
            rightArm = true;
            leftArm = false;
        }else if (gamepad1.x) {
            rightArm = false;
            leftArm = true;
        }

        servoAlignRight.setPosition((rightArm) ? WorldConstants.alignment.ALIGNRIGHTDOWN : WorldConstants.alignment.ALIGNRIGHTUP);
        servoAlignLeft.setPosition((leftArm) ? WorldConstants.alignment.ALIGNLEFTDOWN : WorldConstants.alignment.ALIGNLEFTUP);
        if (gamepad1.dpad_down || gamepad1.dpad_left || gamepad1.dpad_right) {
            if (rightArm && gamepad1.dpad_down && limitRightBack.getState()) {
                speed = 0;
            } else if (rightArm && gamepad1.dpad_left && limitRightSide.getState()) {
                speed = 0.0;
            } else if (leftArm && gamepad1.dpad_down && limitLeftBack.getState()) {
                speed = 0;
            } else if (leftArm && gamepad1.dpad_right && limitRightBack.getState()) {
                speed = 0;
            } else {
                if (gamepad1.dpad_down) {
                    speed = 0.6;
                } else {
                    speed = 0.7;
                }
            }
            robotHandler.drive.mecanum.setMecanum(Math.toRadians((gamepad1.dpad_down) ? 270 : ((gamepad1.dpad_left) ? 180 : 0)), speed, 0, 1.0);
            if ((leftArm && limitLeftSide.getState()) || (rightArm && limitRightSide.getState())) {
                servoFlipL.setPosition(WorldConstants.flip.leftUp);
                servoFlipR.setPosition(WorldConstants.flip.rightUp);
            }

        } else {
            if (thirdPersonOn) {
                robotHandler.drive.mecanum.updateMecanumThirdPerson(gamepad1, (gamepad1.right_stick_button || gamepad1.left_stick_button) ? .5 : 1.0, Math.toRadians(gyro - thirdPersonCal));
            } else {
                robotHandler.drive.mecanum.updateMecanum(gamepad1, (gamepad1.right_stick_button || gamepad1.left_stick_button) ? 0.7 : 1.0, 0);
            }
        }

        collectorSpeed = (gamepad1.right_trigger > 0.10) ? gamepad1.right_trigger : (gamepad1.left_trigger > 0.10) ? -gamepad1.left_trigger : 0;
        motorCollectRight.setPower((gamepad1.y) ? 0.7 : collectorSpeed);
        motorCollectLeft.setPower((gamepad1.y) ? 0.7 : -collectorSpeed);
        motorRelic.setPower((gamepad2.b) ? gamepad2.left_stick_y / 3 : gamepad2.left_stick_y);

        if (gamepad2.right_bumper) {
            servoRelicGrab.setPosition(WorldConstants.relic.grabClose);
        }
        if (gamepad2.left_bumper) {
            servoRelicGrab.setPosition(WorldConstants.relic.grabOpen);
        }
        if (gamepad2.right_trigger > 0.2) {
            servoRelicTurn.setPosition(WorldConstants.relic.turnStraight);
        }
        if (gamepad2.left_trigger > 0.2) {
            servoRelicTurn.setPosition(WorldConstants.relic.turnDown);
        }


        if (gamepad2.y) {
            servoFlipL.setPosition(WorldConstants.flip.leftUp);
            servoFlipR.setPosition(WorldConstants.flip.rightUp);
        } else if (gamepad2.x) {
            servoFlipL.setPosition(WorldConstants.flip.leftFlat);
            servoFlipR.setPosition(WorldConstants.flip.rightFlat);
        } else if (gamepad2.a) {
            servoFlipL.setPosition(WorldConstants.flip.leftDown);
            servoFlipR.setPosition(WorldConstants.flip.rightDown);
        } else if (gamepad2.b){
            servoFlipL.setPosition(WorldConstants.flip.leftFlatAuto);
            servoFlipR.setPosition(WorldConstants.flip.rightFlatAuto);
        }
        if (gamepad2.dpad_left || gamepad2.dpad_right) {
            autoLiftOn = true;
        }
        if (gamepad2.dpad_up || gamepad2.dpad_down) {
            autoLiftOn = false;
        }
        if (gamepad2.x && gamepad2.y) {
            servoJewelHit.setPosition(WorldConstants.auto.jewel.JEWELHITCENTER);
            servoJewel.setPosition(WorldConstants.auto.jewel.JEWELDOWN);
        } else {
            servoJewelHit.setPosition(WorldConstants.auto.jewel.JEWELHITLEFT);
            servoJewel.setPosition(WorldConstants.auto.jewel.JEWELUP);
        }

//        if (gamepad2.dpad_right) {
//            liftTarget = 2800;
//        }
//        if (gamepad1.dpad_left) {
//            liftTarget = 0;
//        }
//        if (autoLiftOn) {
//            if (liftTarget > Math.abs(motorLift.getEncoderPosition())) {
//                motorLift.setPower(-1);
//            } else if (liftTarget < Math.abs(motorLift.getEncoderPosition())) {
//                motorLift.setPower(1);
//
//            }
//        } else {
        motorLift.setPower((gamepad2.dpad_up) ? -1 : (gamepad2.dpad_down) ? 1 : 0);
//        }
        if (gamepad2.a && gamepad2.b) {
            servoGlyphStop.setPosition(WorldConstants.flip.stopUp);
        } else {
            servoGlyphStop.setPosition(WorldConstants.flip.stopDown);
        }

//        servoRelicTurn.setPosition(gamepad2.right_stick_y * 0.000001 + servoRelicTurn.servoObject.getPosition());
//        telemetry.addData("Gyro", 0);
//        telemetry.addData("Lift", motorLift.getEncoderPosition());
        telemetry.update();
//            if (collectorRPM.milliseconds() > 500) {
//                prervPos = motorCollectLeft.getEncoderPosition();
//                prervTime = collectorRPMTimer.milliseconds();
//                collectorRPM.reset();
//            }
    }
//    @Override
//    public void runOpMode() throws InterruptedException {
//        config(this);
//        while (!isStarted() && !isStopRequested()) {
//            telemetry.addData("PID", "CALIBRATING");
//            calibration_complete = !navx_device.isCalibrating();
//            if (!calibration_complete) {
//            } else {
//                navx_device.zeroYaw();
//                yawPIDResult = new navXPIDController.PIDResult();
//            }
//        }
//        waitForStart();
//
////        boolean bClicked = false;
//        double collectorSpeed = 0;
//        double PIDrotationOut = 0;
//        boolean PIDON = false;
//        boolean autoLiftOn = false;
//        ElapsedTime collectorRPMTimer = new ElapsedTime();
//        ElapsedTime collectorRPM = new ElapsedTime();
//        double prervPos = motorCollectLeft.getEncoderPosition();
//        double prervTime = collectorRPMTimer.milliseconds();
//        double liftTarget = 0;
//        while (opModeIsActive()) {
////            double encoderDif = motorCollectLeft.getEncoderPosition() - prervPos;
////            double timeDif = collectorRPMTimer.milliseconds() - prervTime;
////            double RPM = Math.abs((60000 / timeDif) * encoderDif);
////            telemetry.addData("Encoder Val Right", motorCollectRight.getEncoderPosition());
////            telemetry.addData("Encoder Val Left", motorCollectLeft.getEncoderPosition());Ad
////            telemetry.addData("Collector RPM", RPM);
//            //PID
//            if (PIDON&&gamepad1.right_stick_x == 0) {
//                if (yawPIDController.isNewUpdateAvailable(yawPIDResult)) {
//                    if (yawPIDResult.isOnTarget()) {
//                        PIDrotationOut = 0.0;
//                    } else {
//                        PIDrotationOut = yawPIDResult.getOutput();
//                    }
//                }
//            } else {
//                yawPIDController.setSetpoint(navx_device.getYaw());
//                PIDrotationOut = 0;
//            }
//            if (gamepad1.right_bumper) {
//                PIDON = false;
//            }
//            if (gamepad1.left_bumper) {
//                PIDON = true;
//            }
//            if (!PIDON) {
//                PIDrotationOut = 0;
//            }
//
//            robotHandler.drive.mecanum.updateMecanum(gamepad1, (gamepad1.right_bumper) ? 0.7 : 1.0, PIDrotationOut);
//            collectorSpeed = (gamepad1.right_trigger > 0.10) ? gamepad1.right_trigger : (gamepad1.left_trigger > 0.10) ? -gamepad1.left_trigger : 0;
//            motorCollectRight.setPower(collectorSpeed);
//            motorCollectLeft.setPower(-collectorSpeed);
//            motorRelic.setPower((gamepad2.b)?gamepad2.left_stick_y/3:gamepad2.left_stick_y);
//
//            if (gamepad2.right_bumper) {
//                servoRelicGrab.setPosition(WorldConstants.relic.grabClose);
//            }
//            if (gamepad2.left_bumper) {
//                servoRelicGrab.setPosition(WorldConstants.relic.grabOpen);
//            }
//            if (gamepad2.right_trigger > 0.2) {
//                servoRelicTurn.setPosition(WorldConstants.relic.turnStraight);
//            }
//            if (gamepad2.left_trigger > 0.2) {
//                servoRelicTurn.setPosition(WorldConstants.relic.turnDown);
//            }
//            if (gamepad1.y) {
//                servoAlignRight.setPosition(WorldConstants.alignment.ALIGNRIGHTDOWN);
//            } else {
//                servoAlignRight.setPosition(WorldConstants.alignment.ALIGNRIGHTUP);
//            }
//            if (gamepad1.x) {
//                servoAlignLeft.setPosition(WorldConstants.alignment.ALIGNLEFTDOWN);
//            } else {
//                servoAlignLeft.setPosition(WorldConstants.alignment.ALIGNLEFTUP);
//            }
//            if (gamepad2.y) {
//                servoFlipL.setPosition(WorldConstants.flip.leftUp);
//                servoFlipR.setPosition(WorldConstants.flip.rightUp);
//            } else if (gamepad2.x) {
//                servoFlipL.setPosition(WorldConstants.flip.leftFlat);
//                servoFlipR.setPosition(WorldConstants.flip.rightFlat);
//            } else if (gamepad2.a) {
//                servoFlipL.setPosition(WorldConstants.flip.leftDown);
//                servoFlipR.setPosition(WorldConstants.flip.rightDown);
//            }
//            if (gamepad2.dpad_left || gamepad2.dpad_right) {
//                autoLiftOn = true;
//            }
//            if (gamepad2.dpad_up || gamepad2.dpad_down) {
//                autoLiftOn = false;
//            }
//            if (gamepad2.x&&gamepad2.y){
//                servoJewelHit.setPosition(WorldConstants.auto.jewel.JEWELHITCENTER);
//                servoJewel.setPosition(WorldConstants.auto.jewel.JEWELDOWN);
//            } else {
//                servoJewelHit.setPosition(WorldConstants.auto.jewel.JEWELHITLEFT);
//                servoJewel.setPosition(WorldConstants.auto.jewel.JEWELUP);
//            }
//
//            if (gamepad2.dpad_right) {
//                liftTarget = 2800;
//            }
//            if (gamepad1.dpad_left) {
//                liftTarget = 0;
//            }
//            if (autoLiftOn) {
//                if (liftTarget > Math.abs(motorLift.getEncoderPosition())) {
//                    motorLift.setPower(-1);
//                } else if (liftTarget < Math.abs(motorLift.getEncoderPosition())) {
//                    motorLift.setPower(1);
//
//                }
//            } else {
//                motorLift.setPower((gamepad2.dpad_up) ? -1 : (gamepad2.dpad_down) ? 1 : 0);
//            }
//            if (gamepad2.a&&gamepad2.b){
//                servoGlyphStop.setPosition(WorldConstants.flip.stopUp);
//            } else{
//                servoGlyphStop.setPosition(WorldConstants.flip.stopDown);
//            }
//
//            servoRelicTurn.setPosition(gamepad2.right_stick_y*0.000001+servoRelicTurn.servoObject.getPosition());
//            telemetry.addData("PID", PIDON);
//            telemetry.addData("Gyro", navx_device.getYaw());
//            telemetry.addData("Lift", motorLift.getEncoderPosition());
//            telemetry.update();
////            if (collectorRPM.milliseconds() > 500) {
////                prervPos = motorCollectLeft.getEncoderPosition();
////                prervTime = collectorRPMTimer.milliseconds();
////                collectorRPM.reset();
////            }
//
//        }
//    }
}
