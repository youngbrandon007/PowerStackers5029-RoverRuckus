package org.firstinspires.ftc.teamcode.PSRobotPackage.lib.Hardware;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.PSRobotPackage.lib.PSEnum;
import org.firstinspires.ftc.teamcode.PSRobotPackage.lib.PSResources;
import org.firstinspires.ftc.teamcode.PSRobotPackage.lib.Utils.PSGeneralUtils;

import java.util.HashMap;

/**
 * Created on 9/13/2017.
 */

public class PSServo {

    private PSResources resources;
    /**
     * the name of the servo used for hardware mapping
     */
    private String servoName;
    /**
     * the total rotation of the servo
     */
    private double totalRotation;
    /**
     * the init position of the servo
     */
    private double initPos;

    private Servo servoObject;

    private PSEnum.ServoType servoType;

    private HashMap<String, Double> position;

    /**
     * PowerStackers servo constructor
     *
     * @param r             The resources accessible to the servo
     * @param name          The name of the servo used to Hardware map
     * @param totalRotation Th
     * @param init          The init position for the servo to be initialized to
     * @param goToInit      whether the servo should go to the init position in initialization
     */
    public PSServo(PSResources r, String name, double totalRotation, double init, boolean goToInit) {
        resources = r;
        servoName = name;
        this.totalRotation = totalRotation;
        initPos = init;
        servoObject = resources.hardwareMap.get(Servo.class, servoName);
        if (goToInit) servoObject.setPosition(initPos);
    }

    /**
     * PowerStackers servo constructor
     *
     * @param r          The resources accessible to the servo
     * @param name       The name of the servo used to Hardware map
     * @param servoModel The model of the servo, used to calculate the total rotation and degrees
     * @param init       The init position for the servo to be initialized to
     * @param names      The names of the position list
     * @param positions  the positions for the position list
     * @param goToInit   whether the servo should go to the init position in initialization
     */
    public PSServo(PSResources r, String name, PSEnum.ServoTotalRotation servoModel, double init, String[] names, double[] positions, boolean goToInit) {
        this(r, name, servoModel.getRotation(), init, goToInit);
        this.position = PSGeneralUtils.arraysToHashMap(names, positions);
    }

    /**
     * Sets a position to the servo from 0-1
     *
     * @param position the set position
     */
    public void setPosition(double position) {
        servoObject.setPosition(position);
    }

    /**
     * Sets an angle to the servo from 0-max range
     *
     * @param degrees angle to set the servo to
     */
    public void setDegrees(double degrees) {


        double position = Range.clip(degrees / totalRotation, 0, 1);


        setPosition(position);
    }

    /**
     * Uses a set names of positions in the constructor to set the position
     *
     * @param posName The name of the specified positions in the list
     */
    public void setPosition(String posName) {
        if (position.containsKey(posName))
            servoObject.setPosition(position.get(posName));
    }

    /**
     * The function sets the servo's position and moves at a specified speed to that position
     *
     * @param position The position that the servo is set to
     * @param speed    The speed that a servo should move at from 0-1, 1 is max
     */
    public void setPosition(double position, double speed) {
        if (speed == 1) {
            setPosition(position);
        } else {
            //TODO finish this
        }
    }

    /**
     * The function sets the servo's angle and moves at a specified speed to that angle
     *
     * @param degrees The angle that the servo is set to
     * @param speed   The speed that a servo should move at from 0-1, 1 is max
     */
    public void setDegrees(double degrees, double speed) {


        double position = Range.clip(degrees / totalRotation, 0, 1);

        //TODO finish this

        setPosition(position, speed);
    }

}
