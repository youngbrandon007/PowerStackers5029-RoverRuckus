package org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.Drive;

import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PSEnum;
import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PSResources;

/**
 * Created by Brandon on 10/21/2017.
 */

public class PSTankDrive extends PSDriveAbstract {

    PSTankDrive(PSResources r) {
        super(r);
    }

    public void update(double leftPower, double rightPower) {
        setMotor(PSEnum.MotorLoc.LEFT, leftPower, false);
        setMotor(PSEnum.MotorLoc.RIGHT, rightPower, false);
    }

    public void setPower(double leftPower, double rightPower) {
        setMotor(PSEnum.MotorLoc.LEFT, leftPower, true);
        setMotor(PSEnum.MotorLoc.RIGHT, rightPower, true);
    }

//    public void encoderDrive(double speed, String distance, double wheelSize) {
//        PSEnum.MotorValueType motorValueType = getUnit(distance);
//        double value = getVal(distance);
//        if (motorValueType == PSEnum.MotorValueType.COUNTS) {
//            encoderDriveCounts(speed, (int) value);
//        } else {
//            encoderDriveDist(speed, distance, wheelSize);
//
//        }
//    }

//    private void encoderDriveCounts(double speed, int counts) {
//        if (resources.opMode.opModeIsActive()) {
//            if (PSStaticFunction.isPositive(speed) != PSStaticFunction.isPositive(counts)) {
//                counts = -counts;
//            }
//
//            startEncoderDrive(PSEnum.MotorLoc.LEFT, speed, counts);
//            startEncoderDrive(PSEnum.MotorLoc.RIGHT, speed, counts);
//
//            while (resources.opMode.opModeIsActive() && isBusy()){
//                resources.feedBack.update();
//            }
//
//            stop();
//            isBusy();
//            resources.feedBack.update();
//            stopEncoderDrive(PSEnum.MotorLoc.LEFT);
//            stopEncoderDrive(PSEnum.MotorLoc.RIGHT);
//
//        }
//    }

//    private void encoderDriveDist(double speed, String distance, double wheelSize) {
//        PSEnum.MotorValueType motorValueType = getUnit(distance);
//        double value = getVal(distance);
//        int counts = PSStaticFunction.distToCounts(value, motorValueType, wheelSize, getDriveCPR());
//        String countsSring = counts+"ct";
//        encoderDrive(speed, countsSring, wheelSize);
//    }
}
