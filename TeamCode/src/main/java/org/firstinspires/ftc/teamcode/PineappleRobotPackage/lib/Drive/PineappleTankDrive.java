package org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.Drive;

import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PineappleEnum;
import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PineappleResources;

/**
 * Created by Brandon on 10/21/2017.
 */

public class PineappleTankDrive extends PineappleDriveAbstract {

    PineappleTankDrive(PineappleResources r) {
        super(r);
    }

    public void update(double leftPower, double rightPower) {
        setMotor(PineappleEnum.MotorLoc.LEFT, leftPower, false);
        setMotor(PineappleEnum.MotorLoc.RIGHT, rightPower, false);
    }

    public void setPower(double leftPower, double rightPower) {
        setMotor(PineappleEnum.MotorLoc.LEFT, leftPower, true);
        setMotor(PineappleEnum.MotorLoc.RIGHT, rightPower, true);
    }

//    public void encoderDrive(double speed, String distance, double wheelSize) {
//        PineappleEnum.MotorValueType motorValueType = getUnit(distance);
//        double value = getVal(distance);
//        if (motorValueType == PineappleEnum.MotorValueType.COUNTS) {
//            encoderDriveCounts(speed, (int) value);
//        } else {
//            encoderDriveDist(speed, distance, wheelSize);
//
//        }
//    }

//    private void encoderDriveCounts(double speed, int counts) {
//        if (resources.opMode.opModeIsActive()) {
//            if (PineappleStaticFunction.isPositive(speed) != PineappleStaticFunction.isPositive(counts)) {
//                counts = -counts;
//            }
//
//            startEncoderDrive(PineappleEnum.MotorLoc.LEFT, speed, counts);
//            startEncoderDrive(PineappleEnum.MotorLoc.RIGHT, speed, counts);
//
//            while (resources.opMode.opModeIsActive() && isBusy()){
//                resources.feedBack.update();
//            }
//
//            stop();
//            isBusy();
//            resources.feedBack.update();
//            stopEncoderDrive(PineappleEnum.MotorLoc.LEFT);
//            stopEncoderDrive(PineappleEnum.MotorLoc.RIGHT);
//
//        }
//    }

//    private void encoderDriveDist(double speed, String distance, double wheelSize) {
//        PineappleEnum.MotorValueType motorValueType = getUnit(distance);
//        double value = getVal(distance);
//        int counts = PineappleStaticFunction.distToCounts(value, motorValueType, wheelSize, getDriveCPR());
//        String countsSring = counts+"ct";
//        encoderDrive(speed, countsSring, wheelSize);
//    }
}
