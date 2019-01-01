package org.firstinspires.ftc.teamcode.roverRuckus.Robot_r3;

import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.qualcomm.hardware.motors.NeveRest20Gearmotor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

public class DriveConstants_r3 {

    private DriveConstants_r3() {

    }


    public static final MotorConfigurationType MOTOR_CONFIG =
            MotorConfigurationType.getMotorType(NeveRest20Gearmotor.class);
    public static final double TICKS_PER_REV = MOTOR_CONFIG.getTicksPerRev();

    public static double WHEEL_RADIUS = 4; // in
    public static double GEAR_RATIO = 20; // output/input
    public static double TRACK_WIDTH = 15; // in

    public static DriveConstraints BASE_CONSTRAINTS = new DriveConstraints(30.0, 30.0, Math.PI / 2, Math.PI / 2);

    public static double kV = 0.01;
    public static double kA = 0.01;
    public static double kStatic = 0;

}
