package org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.Sensors;

import com.qualcomm.robotcore.hardware.GyroSensor;

import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PSEnum;
import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PSResources;
import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PSSensor;

/**
 * Created by ftcpi on 8/7/2017.
 */

public class PSGyroSensor extends PSSensor {
    public GyroSensor gyroSensor;
    private PSResources resources;
    public PSGyroSensor(String name, PSResources pineappleResources){
        resources = pineappleResources;
        makeSensor(name, pineappleResources);
    }
    @Override
    public void makeSensor(String name, PSResources pineappleResources) {
        sensorName = name;
        gyroSensor = resources.hardwareMap.gyroSensor.get(sensorName);
    }
    public void GSResetZInt () {
        gyroSensor.resetZAxisIntegrator();
    }
    public void GSCalib(){
        gyroSensor.calibrate();
    }
    public boolean GSIsCalib(){
        return gyroSensor.isCalibrating();
    }
    public String GSStatus(){
        return gyroSensor.status();
    }
    public double getValue(PSEnum.PineappleSensorEnum pineappleSensorEnum) {
        switch (pineappleSensorEnum) {
            case GSX:
                return gyroSensor.rawX();
            case GSHEADING:
                return gyroSensor.getHeading();
            case GSROTATIONFRACTION:
                return gyroSensor.getRotationFraction();
            case GSY:
                return gyroSensor.rawY();
            case GSZ:
                return gyroSensor.rawZ();
            default:
                return 0;
        }
    }
}
