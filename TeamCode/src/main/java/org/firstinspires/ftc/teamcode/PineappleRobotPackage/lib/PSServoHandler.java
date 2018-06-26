package org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib;

/**
 * Created by Brandon on 9/13/2017.
 */

public class PSServoHandler {

    private PSResources resources;

    public PSServoHandler(PSResources r) {
        resources = r;
    }

    public PSServo newLimitServo(String name, double pos, double init) {
        return new PSServo(resources, name, PSEnum.ServoType.LIMIT, pos, init, true);
    }

    public PSServo newContinuousServo(String name, double init){
        return new PSServo(resources, name, PSEnum.ServoType.CONTINUOUS, 1, init, true);
    }
}
