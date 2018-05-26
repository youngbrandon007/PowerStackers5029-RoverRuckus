package org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Brandon on 6/19/2017.
 */

public class PineappleMotor {

    //Motor Properties

    public PineappleEnum.MotorLoc motorLoc = PineappleEnum.MotorLoc.NONE;

    public double maxPower = 1;
    public double minPower = -1;

    public double defaultPower = 0;
    public double cpr;
    public double scaleBy = 1;
    public PineappleEnum.MotorType motorType = PineappleEnum.MotorType.UNDI;

    public boolean exponetional = false;

    public boolean doDeadArea = false;

    //Dead Area Array

    private final double[] deadAreaArray = {0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.8, 0.9, 1.0};

    //Motor Object

    public DcMotor motorObject;
    public String motorName;


    //Resources

    private PineappleResources resources;


    //Constructor

    public PineappleMotor(PineappleResources res, String name, double powerMin, double powerMax, double powerDefault, double scale, boolean exp, boolean deadArea, PineappleEnum.MotorLoc loc, PineappleEnum.MotorType type) {
        resources = res;
        motorLoc = loc;
        maxPower = powerMax;
        minPower = powerMin;
        defaultPower = powerDefault;
        scaleBy = scale;
        exponetional = exp;
        doDeadArea = deadArea;
        motorName = name;
        motorType = type;
        cpr = motorTypeToCPR(type);
        mapMotor();
    }

    private void mapMotor() {
        motorObject = resources.hardwareMap.dcMotor.get(motorName);
        setupEncoder();
    }

    private void setupEncoder(){
        motorObject.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        resources.linearOpMode.idle();

        motorObject.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    ///////////////////////////
    //Drive Encoder Functions//
    ///////////////////////////

    private double motorTypeToCPR(PineappleEnum.MotorType type){
        switch (type) {
            case NEV60:
                return PineappleRobotConstants.NEV60CPR;
            case NEV40:
                return PineappleRobotConstants.NEV40CPR;
            case NEV20:
                return PineappleRobotConstants.NEV20CPR;
            case NEV3_7:
                return PineappleRobotConstants.NEV3_7CPR;
            case UNDI:
                return PineappleRobotConstants.TETRIXCPR;
            default:
                return 0;
        }
    }

    public double getEncoderDistance(double wheelSize){
        double rotations = getEncoderPosition()/motorTypeToCPR(motorType);
        return wheelSize*Math.PI*rotations;
    }

    public double getEncoderPosition(){
        return motorObject.getCurrentPosition();
    }

    //Function called by user

    public void encoderDrive(double speed, double value, PineappleEnum.MotorValueType motorValueType, double wheelSize) {
        if (motorValueType == PineappleEnum.MotorValueType.COUNTS) {
            encoderDriveCounts(speed, (int) value);
        } else {
            encoderDriveDist(speed, value, wheelSize, motorValueType);

        }
    }

    //Main encoder drive function

    private void encoderDriveCounts(double speed, int counts){
        int target;
        if(resources.linearOpMode.opModeIsActive()){
            if (PineappleStaticFunction.isPositive(speed) != PineappleStaticFunction.isPositive(counts)){counts=-counts;}

            target = motorObject.getCurrentPosition() + counts;

            motorObject.setTargetPosition(target);
            motorObject.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            motorObject.setPower(speed);

            while (resources.linearOpMode.opModeIsActive() && motorObject.isBusy()){
                resources.feedBack.sayFeedBack(motorName + " encoder", motorObject.getCurrentPosition());
            }

            motorObject.setPower(0);

            motorObject.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    //Drive dist
    //
    //input distance for wheel to move

    private void encoderDriveDist(double speed, double inches, double wheelSize, PineappleEnum.MotorValueType motorValueType ){
        int counts = PineappleStaticFunction.distToCounts(inches, motorValueType,wheelSize, cpr);
        encoderDriveCounts(speed, counts);
    }

    //Encoder control functions
    //
    //start and stop function for motor

    public void encoderStart(double speed, int counts){
        int target;
        if(resources.linearOpMode.opModeIsActive()){
            if (PineappleStaticFunction.isPositive(speed) != PineappleStaticFunction.isPositive(counts)){counts=-counts;}
            target = motorObject.getCurrentPosition() + counts;

            motorObject.setTargetPosition(target);
            motorObject.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            motorObject.setPower(speed);
        }
    }

    public void encoderStop(){
        motorObject.setPower(0);
        motorObject.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public boolean encodersBusy(){
        return motorObject.isBusy();
    }


    //Set Power Functions
    //
    //set Power function for setting the power manually


    public void setPower(double power){
        motorObject.setPower(clip(power));
    }

    //Update Function
    //
    //these are for telemetry update function

    public double update(double power) {
        power = fixValue(power);
        resources.feedBack.sayFeedBack(motorName, power);
        motorObject.setPower(power);
        return power;
    }

    public double update(boolean on) {
        if (on) return update(maxPower);
        else return update(defaultPower);
    }

    public double update(boolean forward, boolean backward) {
        if (forward) return update(maxPower);
        else if (backward) return update(minPower);
        else return update(defaultPower);
    }

    //Private Function
    //
    //these are used to calculate the output to the motor for Tele such as scaling and range

    private double fixValue(double input) {
        input = scale(input);
        input = deadArea(input);
        input = clip(input);
        return input;
    }

    private double deadArea(double input) {
        if (doDeadArea) {
            boolean pos = true;
            if (input < 0) {
                pos = false;
            }
            input = Math.abs(input);

            double last = 10000;
            double now = 0;
            double output = 0;

            for (int i = 0; i < deadAreaArray.length; i++) {
                now = deadAreaArray[i] - input;
                now = Math.abs(now);
                if (now < last) {
                    last = now;
                    output = deadAreaArray[i];
                }
            }

            if (pos == false) {
                output = -output;
            }

            return output;
        } else {
            return input;
        }
    }

    private double scale(double input) {
        input = input * scaleBy;
        if (exponetional) input = input * input;
        return input;
    }

    private double clip(double input) {
        input = Range.clip(input, minPower, maxPower);
        input = Range.clip(input, -1, 1);
        return input;
    }

}
