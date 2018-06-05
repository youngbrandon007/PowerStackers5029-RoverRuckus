package org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib;

/**
 * Created by Brandon on 9/13/2017.
 */

public class PineappleServoHandler {

    private PineappleResources resources;

    public PineappleServoHandler(PineappleResources r) {
        resources = r;
    }

    public PineappleServo newLimitServo(String name, double pos, double init) {
        return new PineappleServo(resources, name, PineappleEnum.ServoType.LIMIT, pos, init);
    }

    public PineappleServo newContinuousServo(String name, double init){
        return new PineappleServo(resources, name, PineappleEnum.ServoType.CONTINUOUS, 1, init);
    }
}
