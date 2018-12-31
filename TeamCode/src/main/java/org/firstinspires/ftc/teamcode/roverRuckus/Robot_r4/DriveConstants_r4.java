package org.firstinspires.ftc.teamcode.roverRuckus.Robot_r4;

import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.qualcomm.hardware.motors.NeveRest20Gearmotor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

public class DriveConstants_r4 {

    private DriveConstants_r4() {

    }


    public static final MotorConfigurationType MOTOR_CONFIG =
            MotorConfigurationType.getMotorType(NeveRest20Gearmotor.class);
    private static final double TICKS_PER_REV = MOTOR_CONFIG.getTicksPerRev();

    public static double WHEEL_RADIUS = 4; // in
    public static double GEAR_RATIO = 20; // output/input
    public static double TRACK_WIDTH = 15; // in

    public static DriveConstraints BASE_CONSTRAINTS;// = new DriveConstraints(30.0, 30.0, Math.PI / 2, Math.PI / 2);

    public static double kV = 0.01;
    public static double kA = 0;
    public static double kStatic = 0;


    public static double encoderTicksToInches(int ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    public static double rpmToVelocity(double rpm) {
        return rpm * DriveConstants_r4.GEAR_RATIO * 2 * Math.PI * DriveConstants_r4.WHEEL_RADIUS / 60.0;
    }
}
