package org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Brandon on 9/13/2017.
 */

public class PineappleServo {

    private PineappleResources resources;

    public String servoName;

    public double totalRotation;

    public double initPos;

    public Servo servoObject;

    public PineappleEnum.ServoType servoType;

    public PineappleServo(PineappleResources r, String name, PineappleEnum.ServoType type, double rotation, double init){
        resources = r;
        servoName = name;
        servoType = type;
        totalRotation = rotation;
        initPos = init;

        servoObject  = resources.hardwareMap.servo.get(servoName);
        servoObject.setPosition(initPos);
    }

    public void setPosition(double position){
        servoObject.setPosition(position);
    }

    public void setDegrees(double degrees){


        double position = clipDegrees(degrees/totalRotation);


        setPosition(position);
    }

    private double clipDegrees(double degrees){
        return Range.clip(degrees, 0 , 1);
    }
}
