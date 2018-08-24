package org.firstinspires.ftc.teamcode.PSRobotPackage.lib;

/**
 * Created by ftcpi on 6/29/2017.
 */


public class PSEnum {

    //Motors
    public enum MotorLoc {
        RIGHT, LEFT, NONE, RIGHTFRONT, LEFTFRONT, RIGHTBACK, LEFTBACK
    }

    public enum MotorValueType {
        INCH, COUNTS, DEGREES, CM, RADIANS, METER, FEET, YARDS
    }

    public enum MotorType {
        NEV60, NEV40, NEV20, NEV3_7, UNDI
    }

    public enum SensorType {
        Touch, Light, Color, Distance
    }

    //Servo
    public enum ServoType {
        CONTINUOUS, LIMIT
    }

    public enum ServoTotalRotation {
        //HiTec
        HS_311(202.5),
        HS_322HD(201),
        HS_485HB(190.5),
        HS_625MG(197),
        HS_645MG(197),
        HS_7980TH(198.5),
        HS_755MG(200.5),
        //Savox
        SA_1230SG(90),
        SC_1256TG(130),
        SC_1258TG(130),
        //Rev
        REVSMARTNOPROG(180),
        REVSMARTPROG(280);
        private double rotation = 180;

        ServoTotalRotation(double deg) {
            this.rotation = deg;
        }
        public double getRotation(){
            return rotation;
        }
    }

    //Vision
    public enum CameraDirection {
        FRONT, BACK
    }
}

