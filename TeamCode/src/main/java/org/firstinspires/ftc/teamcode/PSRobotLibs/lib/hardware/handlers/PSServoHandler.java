package org.firstinspires.ftc.teamcode.PSRobotLibs.lib.hardware.handlers;

import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.PSEnum;
import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.hardware.PSServo;
import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.PSResources;

/**
 * Created by Brandon on 9/13/2017.
 */

public class PSServoHandler {

    /**
     * access to the resources for the servo
     */
    private PSResources resources;

    /**
     * Used to add the servo handler to the robot
     *
     * @param r the resources for the servo
     */
    public PSServoHandler(PSResources r) {
        resources = r;
    }

    /**
     * @param name name of the servo for hardwaremapping
     * @param totalRotation total rotation of the servo being used for degree assignment
     * @param init the init position of the servo
     * @param goToInit whether the servo should be set to the init after hwmap
     */
    public PSServo newServo(String name, double totalRotation, double init, boolean goToInit) {
        return new PSServo(resources,name,totalRotation,init,goToInit);
    }

    /**
     * PowerStacker Servo Constructor, with position storage
     *
     * @param name name of the servo for hardwaremapping
     * @param servoTotalRotation the type of servo being use
     * @param init the init position of the servo
     * @param names the array of names for the positions of the servo
     * @param poses the array of positions for the servo
     * @param goToInit whether the servo should be set to the init after hwmap
     */
    public PSServo newServo(String name, PSEnum.ServoTotalRotation servoTotalRotation, double init, String[] names, double[] poses, boolean goToInit) {
        return new PSServo(resources,name,servoTotalRotation,init,names, poses,goToInit);
    }

}
