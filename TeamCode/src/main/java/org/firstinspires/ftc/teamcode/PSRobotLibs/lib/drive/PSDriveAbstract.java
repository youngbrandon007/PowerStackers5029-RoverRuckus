package org.firstinspires.ftc.teamcode.PSRobotLibs.lib.drive;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.PSRobotLibs.PSSettings;
import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.PSEnum;
import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.hardware.PSMotor;
import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.PSResources;

import java.util.ArrayList;

/**
 * Created by Brandon on 10/21/2017.
 */

public class PSDriveAbstract {

    PSResources resources;

    PSDriveAbstract(PSResources r){
        resources = r;
    }


    public void setMotor(PSEnum.MotorLoc location, double power, boolean direct) {
        ArrayList<PSMotor> motors = resources.storage.getDrivemotors(location);
        for (PSMotor motor : motors) {
            if (direct) {
                motor.setPower(power);
            } else {
                motor.setPower(scalePower(power));
            }
        }
        if(location == PSEnum.MotorLoc.LEFT){
            setMotor(PSEnum.MotorLoc.LEFTFRONT, power, direct);
            setMotor(PSEnum.MotorLoc.LEFTBACK, power, direct);
        }
        if(location == PSEnum.MotorLoc.RIGHT){
            setMotor(PSEnum.MotorLoc.RIGHTFRONT, power, direct);
            setMotor(PSEnum.MotorLoc.RIGHTBACK, power, direct);
        }
    }

    public void stop() {
        setMotor(PSEnum.MotorLoc.LEFT, 0, true);
        setMotor(PSEnum.MotorLoc.RIGHT, 0, true);
        setMotor(PSEnum.MotorLoc.LEFTBACK, 0, true);
        setMotor(PSEnum.MotorLoc.LEFTFRONT, 0, true);
        setMotor(PSEnum.MotorLoc.RIGHTBACK, 0, true);
        setMotor(PSEnum.MotorLoc.RIGHTFRONT, 0, true);
    }

    void startEncoderDrive(PSEnum.MotorLoc location, double power, int counts) {
        ArrayList<PSMotor> motors = resources.storage.getDrivemotors(location);
        for (PSMotor motor : motors) {
            int target = motor.motorObject.getCurrentPosition() + counts;
            motor.motorObject.setTargetPosition(target);
            motor.motorObject.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(power);

        }
    }

    void stopEncoderDrive(PSEnum.MotorLoc location) {
        ArrayList<PSMotor> motors = resources.storage.getDrivemotors(location);
        for (PSMotor motor : motors) {
            motor.motorObject.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    boolean isBusy() {
        boolean output = false;
        ArrayList<PSMotor> motors = resources.storage.getDrivemotors();
        for (PSMotor motor : motors) {
            resources.feedBack.sayFeedBackWithOutUpdate(motor.motorName + " encoder", motor.motorObject.getCurrentPosition());
            if (motor.motorObject.isBusy()) {
                output = true;
            }

        }

        return output;
    }

    PSEnum.MotorType getDriveType() {
        ArrayList<PSMotor> motors = resources.storage.getDrivemotors();
        PSEnum.MotorType motorType = PSEnum.MotorType.UNDI;
        boolean firsttime = true;
        for (PSMotor motor : motors) {
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
        ArrayList<PSMotor> motors = resources.storage.getDrivemotors();
        double lastCPR = 0;
        boolean firsttime = true;
        for (PSMotor motor : motors) {
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

    PSEnum.MotorValueType getUnit(String val) {
        val = val.substring(val.length() - 2);
        switch (val) {
            case "in":
                return PSEnum.MotorValueType.INCH;
            case "ct":
                return PSEnum.MotorValueType.COUNTS;
            case "dg":
                return PSEnum.MotorValueType.DEGREES;
            case "cm":
                return PSEnum.MotorValueType.CM;
            case "rd":
                return PSEnum.MotorValueType.RADIANS;
            case "mt":
                return PSEnum.MotorValueType.METER;
            case "ft":
                return PSEnum.MotorValueType.FEET;
            default:
                return PSEnum.MotorValueType.INCH;
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

        if(PSSettings.driveExponential){
            in = in*in;
            if(!pos){

                in = -in;
            }
        }

        double out = in * PSSettings.driveScaleSpeed;

        return out;
    }
}
