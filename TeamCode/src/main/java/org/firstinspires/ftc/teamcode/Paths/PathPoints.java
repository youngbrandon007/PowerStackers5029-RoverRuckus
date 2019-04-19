package org.firstinspires.ftc.teamcode.Paths;

import android.os.Environment;

import org.opencv.core.Point;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileOutputStream;
import java.io.FileReader;
import java.io.OutputStreamWriter;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

public class PathPoints {
    public static HashMap<Points, PathElement[]> paths = new HashMap<>();

    public static void createPaths(){
        paths.clear();
        String[] points = readPoints();
        for(String point : points){
            String[] pointArray = point.split(":");
            String name = pointArray[0];
            PathElement[] elements = new PathElement[pointArray.length - 1];
            for(int i = 0; i < elements.length; i++){
                String[] elementArray = pointArray[i + 1].split(",");
                elements[i] = new PathElement(Integer.valueOf(elementArray[0]), Double.valueOf(elementArray[1]),  Double.valueOf(elementArray[2]),  Double.valueOf(elementArray[3]),  Double.valueOf(elementArray[4]), Double.valueOf(elementArray[5]));
            }

            paths.put(Points.valueOf(name), elements);
        }
    }

    public static String[] readPoints(){
        try {
            BufferedReader br = new BufferedReader(new FileReader(new File(getPointsPath())));
            String line = br.readLine();
            ArrayList<String> output = new ArrayList<>();
            while (line != null) {
                if(!line.equals("")) output.add(line);
                line = br.readLine();
            }
            return output.toArray(new String[]{});
        }catch(Exception e){
            e.printStackTrace();
            return null;
        }
    }

    private static String getDir(String file){
        return Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_DOWNLOADS) + "/auto/" + file;
    }

    public static String getPointsPath(){
        return getDir("points.txt");
    }

    public static void overridePoints(){
        ArrayList<String> data = new ArrayList<>();
        loadPoints();
        for(Map.Entry<Points, PathElement[]> entry : paths.entrySet()){
            Points p = entry.getKey();
            PathElement[] elements = entry.getValue();

            String append = "";
            append += p.name();

            for(PathElement element : elements){
                append += ":" + element.toString();
            }
            data.add(append);
        }

        try {

            File f = new File(getPointsPath());
            if(f.exists()) f.delete();
            f.createNewFile();
            FileOutputStream fOut = new FileOutputStream(f);
            OutputStreamWriter out = new OutputStreamWriter(fOut);

            for(String line: data){
                out.append(line);
                out.append("\r\n");
            }
            out.close();
            fOut.close();

        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    private static void loadPoints(){

//        //CRATER
//        paths.clear();
//        paths.put(Points.STARTCRATER, new PathElement[]{PathElement.newStart(57, 57, 225)});
//        paths.put(Points.CRATERNOSAMPLE, new PathElement[]{PathElement.newStrafe( 45, 45)});
//        paths.put(Points.CRATERSAMPLE1, new PathElement[]{PathElement.newStrafe( 35, 55), PathElement.newStrafe(30, 50), PathElement.newStrafe(35, 55)});
//        paths.put(Points.CRATERSAMPLE2, new PathElement[]{PathElement.newStrafe( 45, 45), PathElement.newStrafe(40, 40), PathElement.newStrafe(45, 45)});
//        paths.put(Points.CRATERSAMPLE3, new PathElement[]{PathElement.newStrafe( 55, 35), PathElement.newStrafe(50, 30), PathElement.newStrafe(55, 35)});
//        paths.put(Points.CRATERCLAIM, new PathElement[]{PathElement.newRot(135),PathElement.newSpline(20,120, 60)});
//
//        paths.put(Points.PARKFROMDEPOTTOALLIANCECLOSE, new PathElement[]{PathElement.newReverse(true),PathElement.newSpline(12,55, 90)});
//        paths.put(Points.PARKFROMDEPOTTOALLIANCEFAR, new PathElement[]{PathElement.newReverse(true),PathElement.newSpline(45,45, 135), PathElement.newSpline(60, 12, 90)});
//        paths.put(Points.PARKFROMDEPOTTOOPPONET, new PathElement[]{PathElement.newRot(45), PathElement.newSpline(84, 132, 0)});
//
//        paths.put(Points.PARKFROMCRATERLANDERTOALLIANCECLOSE, new PathElement[]{PathElement.newDrive(12)});
//
//        //DEPOT
//        paths.put(Points.STARTDEPOT, new PathElement[]{PathElement.newStart( 57, 87, 135)});
//        paths.put(Points.DEPOTNOSAMPLE, new PathElement[]{PathElement.newStrafe( 45, 99)});
//        paths.put(Points.DEPOTSAMPLE1, new PathElement[]{PathElement.newStrafe( 55, 109), PathElement.newStrafe(50, 114), PathElement.newStrafe(55, 109)});
//        paths.put(Points.DEPOTSAMPLE2, new PathElement[]{PathElement.newStrafe( 45, 99), PathElement.newStrafe(40, 104), PathElement.newStrafe(45, 99)});
//        paths.put(Points.DEPOTSAMPLE3, new PathElement[]{PathElement.newStrafe( 35, 89), PathElement.newStrafe(30, 94), PathElement.newStrafe(35, 89)});
//        paths.put(Points.DEPOTCLAIM, new PathElement[]{PathElement.newStrafe(22, 76), PathElement.newRot(135),PathElement.newSpline(20,120, 60)});
//
//        paths.put(Points.PARKFROMDEPOTLANDERTOALLIANCECLOSE, new PathElement[]{PathElement.newRot(225),PathElement.newSpline(12,55, 270)});
//        paths.put(Points.PARKFROMDEPOTLANDERTOALLIANCEFAR, new PathElement[]{PathElement.newRot(225),PathElement.newStrafe(20,72), PathElement.newRot(315), PathElement.newSpline(60, 12, 270)});
//        paths.put(Points.PARKFROMDEPOTLANDERTOOPPONET, new PathElement[]{PathElement.newRot(45), PathElement.newSpline(86, 130, 25)});

        paths.clear();
//        paths.put(Points.)
        //crater
        paths.put(Points.STARTCRATER, new PathElement[]{PathElement.newStart(57, 57, 225)});
        paths.put(Points.CRATERCLAIM, new PathElement[]{PathElement.newBeginComp(),PathElement.newDrive(4),PathElement.newSpline(33, 60, 135, 225, 250), PathElement.newSpline(9, 90, 90, 225, 270), PathElement.newSpline(20, 120, 70, 270, 250), PathElement.newCloseComp(), PathElement.newSpline(20,120, 250, 250, 250)});//PathElement.newSpline(30, 57, 135)
        paths.put(Points.CRATERSAMPLE1, new PathElement[]{PathElement.newBeginComp(), PathElement.newSpline(14, 90, 270, 250, 270), PathElement.newSpline(26, 64, 315, 270, 250), PathElement.newSpline(30, 46, 225, 250, 225), PathElement.newCloseComp()});
        paths.put(Points.CRATERSAMPLE2, new PathElement[]{PathElement.newBeginComp(), PathElement.newSpline(14, 90, 270, 250, 270), PathElement.newSpline(26, 64, 315, 270, 250), PathElement.newSpline(44, 56, 225, 225, 225), PathElement.newSpline(38, 38, 225, 250, 225), PathElement.newCloseComp()});
        paths.put(Points.CRATERSAMPLE3, new PathElement[]{PathElement.newBeginComp(), PathElement.newSpline(14, 90, 270, 250, 270), PathElement.newSpline(26, 64, 315, 270, 250),  PathElement.newSpline(44, 56, 225, 225, 225), PathElement.newSpline(56, 44, 225, 225, 225),PathElement.newSpline(46, 30, 225, 250, 225), PathElement.newCloseComp()});


        paths.put(Points.STARTDEPOT, new PathElement[]{PathElement.newStart( 57, 87, 135)});
        paths.put(Points.DEPOTSAMPLE1, new PathElement[]{});
        paths.put(Points.DEPOTSAMPLE2, new PathElement[]{});
        paths.put(Points.DEPOTSAMPLE3, new PathElement[]{});
        paths.put(Points.DEPOTCLAIM, new PathElement[]{});

    }

    enum Points{
        STARTDEPOT, DEPOTNOSAMPLE,DEPOTSAMPLE1, DEPOTSAMPLE2, DEPOTSAMPLE3, DEPOTCLAIM,
        PARKFROMDEPOTLANDERTOOPPONET, PARKFROMDEPOTLANDERTOALLIANCECLOSE, PARKFROMDEPOTLANDERTOALLIANCEFAR,

        STARTCRATER, CRATERNOSAMPLE,CRATERSAMPLE1, CRATERSAMPLE2, CRATERSAMPLE3, CRATERCLAIM,
        CRATERDOUBLESAMPLE1, CRATERDOUBLESAMPLE2, CRATERDOUBLESAMPLE3,
        PARKFROMDEPOTTOOPPONET, PARKFROMDEPOTTOALLIANCECLOSE, PARKFROMDEPOTTOALLIANCEFAR,
        PARKFROMCRATERLANDERTOALLIANCECLOSE
    }
}
