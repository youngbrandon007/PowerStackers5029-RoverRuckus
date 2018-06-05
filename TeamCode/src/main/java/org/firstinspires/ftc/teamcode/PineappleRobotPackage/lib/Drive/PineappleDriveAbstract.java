package org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.Drive;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.PineappleRobotPackage.PineappleSettings;
import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PineappleEnum;
import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PineappleMotor;
import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PineappleResources;

import java.util.ArrayList;

/**
 * Created by Brandon on 10/21/2017.
 */

public class PineappleDriveAbstract {

    PineappleResources resources;

    PineappleDriveAbstract(PineappleResources r){
        resources = r;
    }


    public void setMotor(PineappleEnum.MotorLoc location, double power, boolean direct) {
        ArrayList<PineappleMotor> motors = resources.storage.getDrivemotors(location);
        for (PineappleMotor motor : motors) {
            if (direct) {
                motor.setPower(power);
            } else {
                motor.setPower(scalePower(power));
            }
        }
        if(location == PineappleEnum.MotorLoc.LEFT){
            setMotor(PineappleEnum.MotorLoc.LEFTFRONT, power, direct);
            setMotor(PineappleEnum.MotorLoc.LEFTBACK, power, direct);
        }
        if(location == PineappleEnum.MotorLoc.RIGHT){
            setMotor(PineappleEnum.MotorLoc.RIGHTFRONT, power, direct);
            setMotor(PineappleEnum.MotorLoc.RIGHTBACK, power, direct);
        }
    }

    public void stop() {
        setMotor(PineappleEnum.MotorLoc.LEFT, 0, true);
        setMotor(PineappleEnum.MotorLoc.RIGHT, 0, true);
        setMotor(PineappleEnum.MotorLoc.LEFTBACK, 0, true);
        setMotor(PineappleEnum.MotorLoc.LEFTFRONT, 0, true);
        setMotor(PineappleEnum.MotorLoc.RIGHTBACK, 0, true);
        setMotor(PineappleEnum.MotorLoc.RIGHTFRONT, 0, true);
    }

    void startEncoderDrive(PineappleEnum.MotorLoc location, double power, int counts) {
        ArrayList<PineappleMotor> motors = resources.storage.getDrivemotors(location);
        for (PineappleMotor motor : motors) {
            int target = motor.motorObject.getCurrentPosition() + counts;
            motor.motorObject.setTargetPosition(target);
            motor.motorObject.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(power);

        }
    }

    void stopEncoderDrive(PineappleEnum.MotorLoc location) {
        ArrayList<PineappleMotor> motors = resources.storage.getDrivemotors(location);
        for (PineappleMotor motor : motors) {
            motor.motorObject.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    boolean isBusy() {
        boolean output = false;
        ArrayList<PineappleMotor> motors = resources.storage.getDrivemotors();
        for (PineappleMotor motor : motors) {
            resources.feedBack.sayFeedBackWithOutUpdate(motor.motorName + " encoder", motor.motorObject.getCurrentPosition());
            if (motor.motorObject.isBusy()) {
                output = true;
            }

        }

        return output;
    }

    PineappleEnum.MotorType getDriveType() {
        ArrayList<PineappleMotor> motors = resources.storage.getDrivemotors();
        PineappleEnum.MotorType motorType = PineappleEnum.MotorType.UNDI;
        boolean firsttime = true;
        for (PineappleMotor motor : motors) {
            if (firsttime) {
                motorType = motor.motorType;
                firsttime = false;
            } else {
                if (motorType != motor.motorType) {
                    resources.feedBack.sayFeedBackWithOutUpdate("ERROR", "Drive motors incompatiable");
                    resources.feedBack.update();
//                    wait(2000);
                }
                motorType = motor.motorType;
            }
        }
        return motorType;
    }

    double getDriveCPR()  {
        ArrayList<PineappleMotor> motors = resources.storage.getDrivemotors();
        double lastCPR = 0;
        boolean firsttime = true;
        for (PineappleMotor motor : motors) {
            if (firsttime) {
                lastCPR = motor.cpr;
                firsttime = false;
            } else {
                if (lastCPR != motor.cpr) {
                    /////////////////////////
                    //Change to error later//
                    /////////////////////////
                    //
                    //
                    //
                    //
                    //
                    //
                    resources.feedBack.sayFeedBackWithOutUpdate("ERROR", "Drive motor incompatiable");
                    resources.feedBack.update();
                }
                lastCPR = motor.cpr;
            }
        }
        return lastCPR;
    }

    PineappleEnum.MotorValueType getUnit(String val) {
        val = val.substring(val.length() - 2);
        switch (val) {
            case "in":
                return PineappleEnum.MotorValueType.INCH;
            case "ct":
                return PineappleEnum.MotorValueType.COUNTS;
            case "dg":
                return PineappleEnum.MotorValueType.DEGREES;
            case "cm":
                return PineappleEnum.MotorValueType.CM;
            case "rd":
                return PineappleEnum.MotorValueType.RADIANS;
            case "mt":
                return PineappleEnum.MotorValueType.METER;
            case "ft":
                return PineappleEnum.MotorValueType.FEET;
            default:
                return PineappleEnum.MotorValueType.INCH;
        }
    }

    double getVal(String val){
        return Double.parseDouble(val.substring(0, val.length() - 2));
    }


    public double scalePower(double in){

        boolean pos = true;

        if(in < 0){
            pos = false;
        }

        if(PineappleSettings.driveExponential){
            in = in*in;
            if(!pos){

                in = -in;
            }
        }

        double out = in * PineappleSettings.driveScaleSpeed;

        return out;
    }
}
