package org.firstinspires.ftc.teamcode.PSRobotPackage.lib;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import java.util.HashMap;

/**
 * Created by Brandon on 9/13/2017.
 */

public class PSServo {

    private PSResources resources;

    public String servoName;

    public double totalRotation;

    public double initPos;

    public Servo servoObject;

    public PSEnum.ServoType servoType;

    public HashMap<String, Double> position;

    /**
     * @param r The resources accessible to the servo
     * @param name The name of the servo used to Hardware map
     * @param type The type of servo i.e. cont or limited
     * @param totalRotation The //TODO add servo model
     * @param init The init position for the servo to be initialized to
     * @param goToInit whether the servo should go to the init position in initialization
     */
    public PSServo(PSResources r, String name, PSEnum.ServoType type, double totalRotation, double init, boolean goToInit){
        resources = r;
        servoName = name;
        servoType = type;
        this.totalRotation = totalRotation;
        initPos = init;

        servoObject  = resources.hardwareMap.servo.get(servoName);
        if(goToInit)servoObject.setPosition(initPos);
    }

    /**
     * @param r The resources accessible to the servo
     * @param name The name of the servo used to Hardware map
     * @param type The type of servo i.e. cont or limited
     * @param servoModel The model of the servo, used to calculate the total rotation and degrees
     * @param init The init position for the servo to be initialized to
     * @param names The names of the position list
     * @param positions the positions for the position list
     * @param goToInit whether the servo should go to the init position in initialization
     */
    public PSServo(PSResources r, String name, PSEnum.ServoType type, PSEnum.ServoTotalRotation servoModel, double init, String[] names, double[] positions, boolean goToInit){
        resources = r;
        servoName = name;
        servoType = type;
        this.totalRotation = servoModel.getRotation();;
        initPos = init;

        servoObject  = resources.hardwareMap.servo.get(servoName);
        if(goToInit)servoObject.setPosition(initPos);
        this.position = PSStaticFunction.arraysToHashMap(names, positions);
    }

    /**
     * Sets a position to the servo from 0-1
     * @param position the set position
     */
    public void setPosition(double position){
        servoObject.setPosition(position);
    }

    /**
     * Sets an angle to the servo from 0-max range
     * @param degrees angle to set the servo to
     */
    public void setDegrees(double degrees){


        double position = Range.clip(degrees/totalRotation, 0, 1);


        setPosition(position);
    }

    /**
     * Uses a set names of positions in the constructor to set the position
     * @param posName The name of the specified positions in the list
     */
    public void setPosition(String posName){
        if (position.containsKey(posName))
        servoObject.setPosition(position.get(posName));
    }

    /**
     * The function sets the servo's position and moves at a specified speed to that position
     * @param position the position that the servo is set to
     * @param speed The speed that a servo should move at from 0-1, 1 is max
     */
    public void setPosition(double position, double speed){
        if(speed==1){
            setPosition(position);
        } else {
            //TODO finish this
        }
    }

    /**
     * The function sets the servo's angle and moves at a specified speed to that angle
     * @param degrees the angle that the servo is set to
     * @param speed The speed that a servo should move at from 0-1, 1 is max
     */
    public void setDegrees(double degrees, double speed){


        double position = Range.clip(degrees/totalRotation, 0, 1);

        //TODO finish this

        setPosition(position);
    }

}
