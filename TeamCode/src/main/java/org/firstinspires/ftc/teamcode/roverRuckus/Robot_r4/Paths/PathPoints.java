package org.firstinspires.ftc.teamcode.roverRuckus.Robot_r4.Paths;

import java.util.HashMap;

public class PathPoints {
    public static HashMap<Points, PathElement[]> paths = new HashMap<>();

    public final static int START = 0;
    public final static int SPLINE = 1;
    public final static int SPLINEINTERPOLER = 5;
    public final static int STRAFE = 2;
    public final static int ROTATE = 3;
    public final static int DELAY = 4;
    public final static int REVERSE = 6;
    public final static int DRIVE = 7;

    public static void createPaths(){

        //CRATER
        paths.clear();
        paths.put(Points.STARTCRATER, new PathElement[]{PathElement.newStart(55, 55, 225)});
        paths.put(Points.CRATERNOSAMPLE, new PathElement[]{PathElement.newStrafe( 45, 45)});
        paths.put(Points.CRATERSAMPLE1, new PathElement[]{PathElement.newStrafe( 35, 55)});
        paths.put(Points.CRATERSAMPLE2, new PathElement[]{PathElement.newStrafe( 45, 45)});
        paths.put(Points.CRATERSAMPLE3, new PathElement[]{PathElement.newStrafe( 55, 35)});
        paths.put(Points.CRATERCLAIM, new PathElement[]{PathElement.newRot(135),PathElement.newSpline(20,120, 60)});

        paths.put(Points.PARKFROMDEPOTTOALLIANCECLOSE, new PathElement[]{PathElement.newReverse(true),PathElement.newSpline(12,55, 90)});
        paths.put(Points.PARKFROMDEPOTTOALLIANCEFAR, new PathElement[]{PathElement.newReverse(true),PathElement.newSpline(45,45, 135), PathElement.newSpline(60, 12, 90)});
        paths.put(Points.PARKFROMDEPOTTOOPPONET, new PathElement[]{PathElement.newRot(45), PathElement.newSpline(84, 132, 0)});

        paths.put(Points.PARKFROMCRATERLANDERTOALLIANCECLOSE, new PathElement[]{PathElement.newDrive(12)});

        //DEPOT
        paths.put(Points.STARTDEPOT, new PathElement[]{PathElement.newStart( 55, 89, 135)});
        paths.put(Points.DEPOTNOSAMPLE, new PathElement[]{PathElement.newStrafe( 45, 99)});
        paths.put(Points.DEPOTSAMPLE1, new PathElement[]{PathElement.newStrafe( 55, 109)});
        paths.put(Points.DEPOTSAMPLE2, new PathElement[]{PathElement.newStrafe( 45, 99)});
        paths.put(Points.DEPOTSAMPLE3, new PathElement[]{PathElement.newStrafe( 35, 89)});
        paths.put(Points.DEPOTCLAIM, new PathElement[]{PathElement.newStrafe(22, 76), PathElement.newRot(135),PathElement.newSpline(20,120, 60)});

        paths.put(Points.PARKFROMDEPOTLANDERTOALLIANCECLOSE, new PathElement[]{PathElement.newRot(225),PathElement.newSpline(12,55, 270)});
        paths.put(Points.PARKFROMDEPOTLANDERTOALLIANCEFAR, new PathElement[]{PathElement.newRot(225),PathElement.newStrafe(20,72), PathElement.newRot(315), PathElement.newSpline(60, 12, 270)});
        paths.put(Points.PARKFROMDEPOTLANDERTOOPPONET, new PathElement[]{PathElement.newRot(45), PathElement.newSpline(86, 130, 25)});
    }

    enum Points{
        STARTDEPOT, DEPOTNOSAMPLE,DEPOTSAMPLE1, DEPOTSAMPLE2, DEPOTSAMPLE3, DEPOTCLAIM,
        PARKFROMDEPOTLANDERTOOPPONET, PARKFROMDEPOTLANDERTOALLIANCECLOSE, PARKFROMDEPOTLANDERTOALLIANCEFAR,

        STARTCRATER, CRATERNOSAMPLE,CRATERSAMPLE1, CRATERSAMPLE2, CRATERSAMPLE3, CRATERCLAIM,
        PARKFROMDEPOTTOOPPONET, PARKFROMDEPOTTOALLIANCECLOSE, PARKFROMDEPOTTOALLIANCEFAR,
        PARKFROMCRATERLANDERTOALLIANCECLOSE
    }
}
