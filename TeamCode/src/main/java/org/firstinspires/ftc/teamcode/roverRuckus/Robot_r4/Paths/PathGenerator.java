package org.firstinspires.ftc.teamcode.roverRuckus.Robot_r4.Paths;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;

import static org.firstinspires.ftc.teamcode.roverRuckus.Robot_r4.Paths.PathPoints.Points;
import static org.firstinspires.ftc.teamcode.roverRuckus.Robot_r4.Paths.PathPoints.paths;

public class PathGenerator {

    //sample 0-3 (0 default, 1-3 left to right)
    private static String[] dataArray;
    private static TrajectoryBuilder trajectoryBuilder;

    public static Trajectory BuildPath(String data, int samplePos, DriveConstraints constraints){
        PathPoints.createPaths();


        dataArray = data.split(",");
        boolean depotSide = dataValueBool(2);

        //Starting position
        trajectoryBuilder = new TrajectoryBuilder((depotSide) ? paths.get(Points.STARTDEPOT)[0].getPose2d() : paths.get(Points.STARTCRATER)[0].getPose2d(),constraints);

        //delay
        //addElement(PathElement.newDelayFull(dataValueDouble(1)));

        //Sample
        if(dataValueBool(4)){
            Points[] sample = (depotSide) ? new Points[]{Points.DEPOTSAMPLE1, Points.DEPOTSAMPLE2, Points.DEPOTSAMPLE3} : new Points[]{Points.CRATERSAMPLE1, Points.CRATERSAMPLE2, Points.CRATERSAMPLE3};
            addElement(sample[samplePos - 1]);
            //addElement(PathElement.newDelayFull(1.0));
        }else{
            addElement((depotSide) ? Points.DEPOTNOSAMPLE : Points.CRATERNOSAMPLE);
        }

        if(dataValueBool(6)){
            //Claim
            addElement((depotSide) ? Points.DEPOTCLAIM : Points.CRATERCLAIM);
            //Wait for claim
            addElement(PathElement.newDelayFull(1.0));
            //double sample
            if(dataValueBool(5) && depotSide == false){
                Points[] doubleSample = new Points[]{Points.CRATERDOUBLESAMPLE1, Points.CRATERDOUBLESAMPLE2, Points.CRATERDOUBLESAMPLE3};
                addElement(doubleSample[samplePos - 1]);
            }
            //Park
            addElement((dataValueBool(7)) ? Points.PARKFROMDEPOTTOOPPONET : (dataValueBool(8)) ? Points.PARKFROMDEPOTTOALLIANCEFAR : Points.PARKFROMDEPOTTOALLIANCECLOSE);
        }else{
            if(depotSide){
                addElement((dataValueBool(7)) ? Points.PARKFROMDEPOTLANDERTOOPPONET : (dataValueBool(8)) ? Points.PARKFROMDEPOTLANDERTOALLIANCEFAR : Points.PARKFROMDEPOTLANDERTOALLIANCECLOSE);
            }else{
                addElement(Points.PARKFROMCRATERLANDERTOALLIANCECLOSE);
            }
        }
        return trajectoryBuilder.build();
    }
    private static void addElement(Points p){
       addElement(paths.get(p));
    }

    private static void addElement(PathElement[] elements){

        for(PathElement element : elements){
            switch (element.type){
                case PathElement.SPLINEINTERPOLER:
                    trajectoryBuilder.splineTo(element.getPose2d(), element.getInterpolator());
                    break;
                case PathElement.SPLINE:
                    trajectoryBuilder.splineTo(element.getPose2d());
                    break;
                case PathElement.STRAFE:
                    trajectoryBuilder.strafeTo(element.getVector2d());
                    break;
                case PathElement.ROTATE:
                    trajectoryBuilder.turnTo(element.getRot());
                    break;
                case PathElement.DELAY:
                    trajectoryBuilder.waitFor(element.getDelay());
                    break;
                case PathElement.REVERSE:
                    trajectoryBuilder.setReversed(element.reverse());
                    break;
                case PathElement.DRIVE:
                    trajectoryBuilder.forward(element.distance());
                    break;
            }
        }
    }

    private static double dataValueDouble(int index){
        return Double.valueOf(dataArray[index].split("=")[1]);
    }

    private static boolean dataValueBool(int index){
        return Boolean.valueOf(dataArray[index].split("=")[1]);
    }
}
