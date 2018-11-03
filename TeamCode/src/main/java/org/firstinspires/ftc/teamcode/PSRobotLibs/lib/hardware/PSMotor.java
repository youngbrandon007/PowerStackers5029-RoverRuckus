package org.firstinspires.ftc.teamcode.PSRobotLibs.lib.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.PSEnum;
import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.PSResources;
import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.PSRobotConstants;

/**
 * Created by Brandon on 6/19/2017.
 */

public class PSMotor {

    /**
     * The location of the motor in the robot
     */
    public PSEnum.MotorLoc motorLoc;
    /**
     * the max power of the motor
     */
    private double maxPower = 1;
    /**
     * the min power of the motor
     */
    private double minPower = -1;

    /**
     * the defaultPower of the motor
     */
    private double defaultPower = 0;
    /**
     * the counts per revolution of the motor encoder
     */
    public double cpr;
    /**
     * scale for the motor speed
     */
    private double scaleBy = 1;
    /**
     * whether the motor has an exponetial input
     */
    public boolean exponential;
    /**
     * whether the motor has dead area or not
     */
    public boolean doDeadArea;
    /**
     * the gear ratio of the motor from bare to actuator
     */
    private final double gearRatio;
    /**
     * Dead Area Array
     */
    private final double[] deadAreaArray = {0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.8, 0.9, 1.0};
    /**
     * the motor delegate
     */
    public DcMotor motorObject;
    /**
     * the name of the motor for hardware maps
     */
    public String motorName;
    /**
     * resources for the motor so it has access to everything
     */
    private PSResources resources;
    /**
     * var to cache the last power so the the motor isn't set to the same power
     */
    private double cachedPower;

    public PSMotor(PSResources resources, String motorName, double minPower, double maxPower, double defaultPower, double scaleBy, boolean exponential, boolean doDeadArea, PSEnum.MotorLoc motorLoc, double gearRatio) {
        this.resources = resources;
        this.motorLoc = motorLoc;
        this.maxPower = maxPower;
        this.minPower = minPower;
        this.defaultPower = defaultPower;
        this.scaleBy = scaleBy;
        this.exponential = exponential;
        this.doDeadArea = doDeadArea;
        this.motorName = motorName;
        this.gearRatio = gearRatio;
        cpr = getCpr();
        //motorObject = this.resources.hardwareMap.dcMotor.get(motorName);
        motorObject = this.resources.hardwareMap.get(DcMotor.class, motorName);
        setupEncoder();
    }

    private void setupEncoder() {
        motorObject.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorObject.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    ///////////////////////////
    //Drive Encoder Functions//
    ///////////////////////////

    private double getCpr() {
        return PSRobotConstants.NEVERESTPPR * gearRatio;
    }

    public double getEncoderDistance(double wheelSize) {
        double rotations = getEncoderPosition() / cpr;
        return wheelSize * Math.PI * rotations;
    }

    public double getEncoderPosition() {
        return motorObject.getCurrentPosition();
    }


    public void encoderStop() {
        motorObject.setPower(0);
        motorObject.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public boolean encodersBusy() {
        return motorObject.isBusy();
    }


    //Set Power Functions
    //
    //set Power function for setting the power manually


    public void setPower(double power) {
        if (cachedPower != power)
            motorObject.setPower(clip(power));
        cachedPower = power;
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
    //these are used to calculate the output to the motor for WorldTele such as scaling and range

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
        if (exponential) input = input * input;
        return input;
    }

    private double clip(double input) {
        input = Range.clip(input, minPower, maxPower);
        input = Range.clip(input, -1, 1);
        return input;
    }

}
