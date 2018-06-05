package org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.Auto;

import android.os.Environment;
import android.util.Log;

import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.Drive.PineappleDrive;
import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PineappleEnum;
import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PineappleResources;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;

/**
 * Created by young on 8/6/2017.
 */

public class PineappleAutoDrive {

    private PineappleResources resources;

    private PineappleDrive drive;

    /**
     * Constructor of the WorldAuto drive class
     *
     * @param res   Passes the resources for use in methods
     * @param drive gives direct access to the drive methods for ease of use
     */
    public PineappleAutoDrive(PineappleResources res, PineappleDrive drive) {
        resources = res;
        this.drive = drive;
    }

//    public void lineFollow(PineappleSensor color, PineappleEnum.PineappleSensorEnum colorEnum, PineappleSensor sensor, PineappleEnum.PineappleSensorEnum sensorEnum, PineappleEnum.Condition condition, double sensorValue, double power) {
//        if (checkCondition(sensor.getValue(sensorEnum), sensorValue, condition)) {
//
//
//            while (checkCondition(sensor.getValue(sensorEnum), sensorValue, condition) && resources.opMode.opModeIsActive()) {
//
//                double turn = (color.getValue(colorEnum) - .2) * 2;
//
//                double left = 1 - turn;
//                double right = turn;
//
//                drive.setMotor(PineappleEnum.MotorLoc.RIGHT, (right - .2) * power, true);
//                drive.setMotor(PineappleEnum.MotorLoc.LEFT, (left - .2) * power, true);
//
//                resources.feedBack.sayFeedBackWithOutUpdate(color.sensorName, color.getValue(colorEnum));
//                resources.feedBack.sayFeedBack(sensor.sensorName, sensor.getValue(sensorEnum));
//            }
//
//            drive.stop();
//        }
//    }

    /**
     * Turns the robot until a sensor is triggered
     *
     * @param sensor      The sensor to be triggered
     * @param sensorEnum  The type of sensor for id
     * @param condition   When it should stop
     * @param sensorValue what the value of the sensor is when it stops
     * @param rightPower  The power of the right side while turning
     * @param leftPower   The power of the left side while turning
     */
//    public void turnUntil(PineappleSensor sensor, PineappleEnum.PineappleSensorEnum sensorEnum, PineappleEnum.Condition condition, double sensorValue, double rightPower, double leftPower) {
//        if (checkCondition(sensor.getValue(sensorEnum), sensorValue, condition)) {
//
//            double percent = 0;
//            double starting = sensor.getValue(sensorEnum);
//            double now = starting;
//
//            drive.setMotor(PineappleEnum.MotorLoc.RIGHT, rightPower, true);
//            drive.setMotor(PineappleEnum.MotorLoc.LEFT, leftPower, true);
//            while (checkCondition(sensor.getValue(sensorEnum), sensorValue, condition) && resources.opMode.opModeIsActive()) {
//                now = sensor.getValue(sensorEnum);
//                percent = (now - starting) / (sensorValue - starting);
//                if (percent > .85) {
//
//                    drive.setMotor(PineappleEnum.MotorLoc.RIGHT, rightPower / 3, true);
//                    drive.setMotor(PineappleEnum.MotorLoc.LEFT, leftPower / 3, true);
//
//                }
//
//                resources.feedBack.sayFeedBack(sensor.sensorName, sensor.getValue(sensorEnum));
//            }
//
//            drive.stop();
//        }
//    }

    /**
     * @param sensorValue    Method for checking if the sensor has reached goal
     * @param conditionValue The Type of specified operator
     * @param condition      The type of operator the value is check by
     * @return - whether or not it has reached
     */
    private boolean checkCondition(double sensorValue, double conditionValue, PineappleEnum.Condition condition) {
        switch (condition) {
            case EQUAL:
                if (sensorValue == conditionValue) {
                    return false;
                }
                break;
            case LESSTHAN:
                if (sensorValue < conditionValue) {
                    return false;
                }
                break;
            case GREATERTHAN:
                if (sensorValue > conditionValue) {
                    return false;
                }
                break;
        }
        return true;
    }
//
//    public void gyroTurnPID(double degrees, double P, double I, double D, AHRS navx_device) throws InterruptedException {
//
//        final double TARGET_ANGLE_DEGREES = degrees;
//        final double TOLERANCE_DEGREES = 2.0;
//
//        Calendar calendar = Calendar.getInstance();
//        int year = calendar.get(Calendar.YEAR);
//        int month = calendar.get(Calendar.MONTH);
//        int day = calendar.get(Calendar.DATE);
//        int hour = calendar.get(Calendar.HOUR_OF_DAY);
//        int minute = calendar.get(Calendar.MINUTE);
//        int second = calendar.get(Calendar.SECOND);
//        int millisecond = calendar.get(Calendar.MILLISECOND);
//        String full = "" + year + month + day + "_" + hour + minute + second + millisecond;
//        String name = full + "_" + TARGET_ANGLE_DEGREES + "_ " + P + "_" + I + "_" + D + "_";
//        String data = "TIME,OUTPUT,YAW,TARGET";
//        ElapsedTime el = new ElapsedTime();
//
//        double YAW_PID_P = P;
//        double YAW_PID_I = I;
//        double YAW_PID_D = D;
//        navXPIDController yawPIDController = new navXPIDController(navx_device,
//                navXPIDController.navXTimestampedDataSource.YAW);
//
//        /* Configure the PID controller */
//        yawPIDController.setSetpoint(TARGET_ANGLE_DEGREES);
//        yawPIDController.setContinuous(false);
//        yawPIDController.setOutputRange(RelicRecoveryConstants.MIN_MOTOR_OUTPUT_VALUE, RelicRecoveryConstants.MAX_MOTOR_OUTPUT_VALUE);
//        yawPIDController.setTolerance(navXPIDController.ToleranceType.ABSOLUTE, TOLERANCE_DEGREES);
//        yawPIDController.setPID(YAW_PID_P, YAW_PID_I, YAW_PID_D);
//        try {
//            yawPIDController.enable(true);
//
//        /* Wait for new Yaw PID output values, then update the motors
//           with the new PID value with each new output value.
//         */
//
//            final double TOTAL_RUN_TIME_SECONDS = 30.0;
//            int DEVICE_TIMEOUT_MS = 1000;
//            navXPIDController.PIDResult yawPIDResult = new navXPIDController.PIDResult();
//
//            DecimalFormat df = new DecimalFormat("#.###");
//            el.reset();
//            while (resources.opMode.opModeIsActive() &&
//                    !Thread.currentThread().isInterrupted()) {
//                double output = 0;
//                if (yawPIDController.waitForNewUpdate(yawPIDResult, DEVICE_TIMEOUT_MS)) {
//                    if (yawPIDResult.isOnTarget()) {
//                        resources.telemetry.addData("PIDOutput", df.format(0.00));
//                    } else {
//                        output = yawPIDResult.getOutput();
//                        drive.tank.setPower(output, output);
//
//                        resources.telemetry.addData("PIDOutput", df.format(output) + ", " +
//                                df.format(-output));
//                    }
//                } else {
//                /* A timeout occurred */
//                    resources.telemetry.addData("navXRotateOp", "Yaw PID waitForNewUpdate() TIMEOUT.");
//                }
//                resources.telemetry.addData("Yaw", df.format(navx_device.getYaw()));
//
//                data += "\n" + el.milliseconds() + "," + output * 100 + "," + navx_device.getYaw() + "," + TARGET_ANGLE_DEGREES;
//
//                resources.telemetry.update();
//
//            }
//        } finally {
//            navx_device.close();
//            resources.telemetry.addData("LinearOp", "Complete");
//        }
//        writeToFile(name, data);
//
//
//    }

    private void writeToFile(String name, String data) {
        File path = Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_DOWNLOADS);
        File file = new File(path + "/PID", name + ".csv");
        try {
            FileOutputStream stream = new FileOutputStream(file, true);
            stream.write(data.getBytes());
            stream.close();
            Log.i("saveData", "Data Saved");
        } catch (IOException e) {
            Log.e("SAVE DATA", "Could not write file " + e.getMessage());
        }
    }

}