package org.firstinspires.ftc.teamcode.PSRobotLibs.lib.drive;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.PSEnum;
import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.PSResources;
import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.utils.PSGeneralUtils;

import static java.lang.Math.abs;
import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static java.lang.Math.sqrt;

/**
 * Created by Brandon on 10/21/2017.
 */

public class PSMecanumDrive extends PSDriveAbstract {

    PSMecanumDrive(PSResources r) {
        super(r);
    }

    public void update(double leftPower, double rightPower) {
        setMotor(PSEnum.MotorLoc.LEFTFRONT, leftPower, false);
        setMotor(PSEnum.MotorLoc.LEFTBACK, leftPower, false);
        setMotor(PSEnum.MotorLoc.RIGHTFRONT, rightPower, false);
        setMotor(PSEnum.MotorLoc.RIGHTBACK, rightPower, false);
    }

    public void setPower(double leftPower, double rightPower) {
        setMotor(PSEnum.MotorLoc.LEFTFRONT, leftPower, true);
        setMotor(PSEnum.MotorLoc.LEFTBACK, leftPower, true);
        setMotor(PSEnum.MotorLoc.RIGHTFRONT, rightPower, true);
        setMotor(PSEnum.MotorLoc.RIGHTBACK, rightPower, true);
    }

    public void updateMecanumMultiGamepad(Gamepad pad1, double offset1, Gamepad pad2, double offset2, double scale1, double scale2) {
        double angle1 = mecDirectionFromJoystick(pad1) + Math.toRadians(offset1);
        double speed1 = mecSpeedFromJoystick(pad1);
        double rotation1 = mecSpinFromJoystick(pad1);

        double angle2 = mecDirectionFromJoystick(pad2) + Math.toRadians(offset2);
        double speed2 = mecSpeedFromJoystick(pad2);
        double rotation2 = mecSpinFromJoystick(pad2);

        if(Math.abs(speed1) > 0.05 || Math.abs(rotation1) > 0.05){
            setMecanum(angle1, speed1, rotation1, scale1);
        }else{
            setMecanum(angle2, speed2, rotation2, scale2);
        }


    }

    public void updateMecanum(Gamepad pad, double scale) {

        double angle = mecDirectionFromJoystick(pad);
        double speed = mecSpeedFromJoystick(pad);
        double rotation = mecSpinFromJoystick(pad);

        setMecanum(angle, speed, rotation, scale);

    }

    public void updateMecanumCustom(double angle, double speed, double rotation, double scale){
        setMecanum(angle, speed, rotation, scale);
    }

    public void updateMecanum(Gamepad pad, double scale, double PID) {

        double angle = mecDirectionFromJoystick(pad);
        double speed = mecSpeedFromJoystick(pad);
        double rotation = mecSpinFromJoystick(pad)+PID;

        setMecanum(angle, speed, rotation, scale);

    }

    public void updateMecanumThirdPerson(Gamepad pad, double scale, double gyroAngle){
        double angle = mecDirectionFromJoystick(pad);
        double speed = mecSpeedFromJoystick(pad);
        double rotation = mecSpinFromJoystick(pad);

        setMecanumThridPerson(angle, speed, rotation, scale, gyroAngle);
    }

    public void setMecanumThridPerson(double angle, double speed, double rotation, double scale, double gyroAngle){
        setMecanum(angle - gyroAngle, speed, rotation, scale);
    }

    public void setMecanum(double angle, double speed, double rotation, double scale) {
        angle += Math.PI / 4;
        speed *= Math.sqrt(2);

        double sinDir = sin(angle);
        double cosDir = cos(angle);
        double multipliers[] = new double[4];
        multipliers[0] = (speed * sinDir) + rotation;
        multipliers[1] = (speed * cosDir) + rotation;
        multipliers[2] = (speed * -cosDir) + rotation;
        multipliers[3] = (speed * -sinDir) + rotation;

        double largest = abs(multipliers[0]);
        for (int i = 1; i < 4; i++) {
            if (abs(multipliers[i]) > largest)
                largest = abs(multipliers[i]);
        }

        // Only normalize multipliers if largest exceeds 1.0
        if (largest > 1.0) {
            for (int i = 0; i < 4; i++) {
                multipliers[i] = multipliers[i] / largest;
            }
        }


        setMotor(PSEnum.MotorLoc.LEFTFRONT, multipliers[0] * scale, true);
        setMotor(PSEnum.MotorLoc.RIGHTFRONT, multipliers[1] * scale, true);
        setMotor(PSEnum.MotorLoc.LEFTBACK, multipliers[2] * scale, true);
        setMotor(PSEnum.MotorLoc.RIGHTBACK, multipliers[3] * scale, true);

    }

    private double[] getXY(double rad, double counts) {

        double[] xy = new double[2];
        double x = 0;
        double y = 0;

        x = (Math.cos(rad) * counts);
        y = (Math.sin(rad) * counts);
        x = PSGeneralUtils.round(x, 4);
        y = PSGeneralUtils.round(y, 4);
        xy[0] = x;
        xy[1] = y;
        return xy;
    }

    private static double mecSpinFromJoystick(Gamepad pad) {
        return (abs(pad.right_stick_x) > 0.15f)
                ? pad.right_stick_x : 0.0;
    }

    private static double mecDirectionFromJoystick(Gamepad pad) {
        return Math.atan2(-pad.left_stick_y, pad.left_stick_x);
    }

    private static double mecSpeedFromJoystick(Gamepad pad) {
        // If the joystick is close enough to the middle, return a 0 (no movement)
        if (abs(pad.left_stick_x) < 0.15f
                && abs(pad.left_stick_y) < 0.15f) {
            return 0.0;
        } else {
            return sqrt((pad.left_stick_y * pad.left_stick_y)
                    + (pad.left_stick_x * pad.left_stick_x));
        }
    }
}
