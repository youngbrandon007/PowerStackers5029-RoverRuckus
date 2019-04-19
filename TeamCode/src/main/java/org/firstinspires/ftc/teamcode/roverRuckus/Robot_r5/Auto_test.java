package org.firstinspires.ftc.teamcode.roverRuckus.Robot_r5;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.path.heading.SplineInterpolator;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.roverRuckus.Robot_r5.Transition.AutoTransitioner;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name="TransitionTest", group = "r4")
@Disabled
public class Auto_test extends Config_r5 {

    Trajectory trajectory;

    enum Action{
        DRIVETOPICK, PICK, DRIVETOPLACE
    }

    Action action = Action.PICK;

    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    private static final String VUFORIA_KEY = "AS+d9VT/////AAABmctzt2QKCkCHsWckqSrT1KQGWoYW3xOlwF6eKDby0X3s3w/cOZDacTUq9nSp07w99XYiDRBwazpKsyrGpb91tabfndtOpx9EmRnJXteDwU8VXOnWysQIG8PBqHCXmOTYwXCCcJtp6KQIDjw9PrJDlUAit/rWlA5pLD8UdFyFkJiTNaInnUxsRqQQSeUMxKGuf4QcHDh6g72cZ9GjrCH+hdgIFJ7mta711jglu1a8kMKHSD9XbBOhD6DK9tATWVSsVEO4DCuKz61TRpCuqfSc62yXHkWDHanBSG9ca99r+TVrwKE7QCzMDkLSrKW/KHSr+MNPSTuCGnWBGNL4QSmsJ6ec6IqINx7IYWFeUMyFGM4w";

    private VuforiaLocalizer vuforia;

    private TFObjectDetector tfod;

    ArrayList<Mineral> minerals = new ArrayList<>();
    Vector2d pickLocation = new Vector2d(24, 24);

    @Override
    public void init() {
        config(this);
        telemetry.addData("Init", 1);
        telemetry.update();

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        telemetry.addData("Init", 2);
        telemetry.update();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        telemetry.addData("Init", 3);
        telemetry.update();
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        telemetry.addData("Init", 4);
        telemetry.update();
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);

        telemetry.addData("Init", 5);
        telemetry.update();
        AutoTransitioner.transitionOnStop(this, "r5.Tele");

        drive.calibrationPosition = new Vector2d(48, 48);
        gyro.cal = 225;
    }

    @Override
    public void start(){
        tfod.activate();
    }

    @Override
    public void loop() {
        Pose2d est = drive.getEstimatedPose();
        telemetry.addData("pos.x", est.getX());
        telemetry.addData("pos.y", est.getY());
        telemetry.addData("pos.head",  Math.toDegrees(est.getHeading()));
        Pose2d min = getPickLocation(new Vector2d(24,24));
        telemetry.addData("go.x", min.getX());
        telemetry.addData("go.y", min.getY());
        telemetry.addData("go.head", Math.toDegrees(min.getHeading()));
        telemetry.addLine("Best (" + String.valueOf(pickLocation.getX()) + ", " + String.valueOf(pickLocation.getY()) + ")");
        //Minerals finding
        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
        if (updatedRecognitions != null) {
            minerals.clear();

            for(Recognition rec:updatedRecognitions){
                double mx = 0;
                double my = 0;
                double mz = (rec.getLabel().equals(LABEL_GOLD_MINERAL) ? 1 : 1.375);
                double cx = est.getX();
                double cy = est.getY();
                double cz = 12;
                double crx = est.getHeading(); //radians
                double cry = 30; // 0 is looking at wall - degrees
                double cpw = rec.getImageWidth();
                double cph = rec.getImageHeight();
                double cfovax = 70.42;
                double cfovay = 43.3;
                double rx = 0;
                double ry = 0;

                double mpx = ((rec.getRight() + rec.getLeft()) / 2) - (cpw / 2);
                double mpy = ((rec.getTop() + rec.getBottom()) / 2) - (cpw / 2);

//                telemetry.addData("x", mpx);
//                telemetry.addData("y", mpy);

                double ax = (cfovax* mpx/cpw);
                double ay = -((-cfovay* mpy/cph)-20);

//                telemetry.addData("ax", ax);
//                telemetry.addData("ay", ay);

                double relxdis = Math.tan(Math.toRadians((90 - cry) - ay)) * (cz - mz); // relative x distance
                double relydis = Math.tan(Math.toRadians(ax)) * (relxdis);
//                telemetry.addData("relx", relxdis);
//                telemetry.addData("rely", relydis);

                mx = cx + (Math.cos(crx) * relxdis);
                mx = mx + (Math.sin(crx - (Math.PI / 2)) * relydis);

                my = cy + (Math.sin(crx) * relxdis);
                my = my + (Math.cos(crx - (Math.PI / 2)) * relydis);

//                telemetry.addData("COORD x", mx);
//                telemetry.addData("COORD y", my);

                minerals.add(new Mineral(rec, mx , my,  mz));

                telemetry.addLine("M" + String.valueOf(minerals.size()) + " (" + String.valueOf(mx) + ", " + String.valueOf(my) + ") relative:(" + String.valueOf(relxdis) + ", " + String.valueOf(relydis) + ")" + rec.getLabel());
            }

            telemetry.addData("Mineral Count", minerals.size());

            pickLocation = findBestCluster();
        }
        //actions
        switch(action){
            case DRIVETOPICK:
                if (drive.isFollowingTrajectory()) {
                    drive.update();
                }else{
                    action = Action.PICK;
                }
                break;
            case PICK:
                robot.drive.mecanum.updateMecanum(gamepad1, 1.0);
                if(gamepad1.x){
                    drive.calibrationPosition = new Vector2d(48, 48);
                    gyro.cal = 225;
                }
                if(gamepad1.b){
                    trajectory = new TrajectoryBuilder(drive.getEstimatedPose(), new DriveConstraints(50.0, 70.0, 2, 3))
                           // .lineTo(new Vector2d(0, 0),LinearInterpolator(drive.getEstimatedPose().getHeading(), 0))
                            .lineTo(new Vector2d(48,48), new SplineInterpolator(drive.getEstimatedPose().getHeading(), Math.toRadians(225)))
                            .build();

                    drive.followTrajectory(trajectory);

                    action = Action.DRIVETOPLACE;
                }
                if(gamepad1.y){
                    Pose2d go = getPickLocation(pickLocation);

                    trajectory = new TrajectoryBuilder(drive.getEstimatedPose(), new DriveConstraints(50.0, 70.0, 2, 3))
                            // .lineTo(new Vector2d(0, 0),LinearInterpolator(drive.getEstimatedPose().getHeading(), 0))
                            .lineTo(new Vector2d(go.getX(),go.getY()), new SplineInterpolator(drive.getEstimatedPose().getHeading(), go.getHeading()))
                            .build();

                    drive.followTrajectory(trajectory);

                    action = Action.DRIVETOPICK;
                }
                break;
            case DRIVETOPLACE:


                if (drive.isFollowingTrajectory()) {
                    drive.update();
                }else{
                    action = Action.PICK;
                }
                break;
        }

        telemetry.addData("Action", action);
    }

    public Vector2d findBestCluster(){
        Vector2d best = new Vector2d(0, 0);
        double bestScore = 0;
        for(Mineral m : minerals){
            double score = 0;
            for(Mineral check: minerals){
                double disToMin = Math.sqrt(Math.pow(check.x - m.x , 2) + Math.pow(check.y - m.y, 2));
                if(disToMin < 4){
                    score += 10;
                }else if(disToMin < 6){
                    score += 2;
                }
            }
            Pose2d rob = drive.getEstimatedPose();
            double disToRob = Math.sqrt(Math.pow(rob.getX() - m.x , 2) + Math.pow(rob.getY() - m.y, 2));
            double addPoints = (100 - disToRob);
            score += (disToRob > 0) ? disToRob : 0;
            if(score > bestScore){
                bestScore = score;
                best = new Vector2d(m.x, m.y);
            }
        }
        return best;
    }

    public Pose2d getPickLocation(Vector2d mineralLoc){
        Pose2d robotPos = drive.getEstimatedPose();

        double rx = robotPos.getX();
        double ry = robotPos.getY();
        double mx = mineralLoc.getX();
        double my = mineralLoc.getY();

        double angle = Math.atan2(my - ry, mx - rx);
        angle += (angle < 0) ? 2* Math.PI : 0;

        if((rx - mx) == 0){
            //y=-5/6x+71.667
            //x=rx
            double i1x = rx;
            double i1y = (- (5.0/6.0)) * i1x + (71 + (2.0/3.0));

            //y=60
            //x=rx
            double i2x = rx;
            double i2y = 60;

            if(i1y >= 60){
                return new Pose2d(i2x, i2y, angle);
            }else{
                return new Pose2d(i1x, i1y, angle);
            }
        }else{
            double m = (ry - my) / (rx - mx);
            double b = ry - (m * rx);

            //y=-5/6x+71.667
            //y=mx+b
            //mx+b=-5/6x+71.667
            //x(m+5/6)=71.667-b
            //x=(71.667-b)/(m+5/6)

            double i1x = ((71 + (2.0/3.0)) - b) / (m + (5.0/6.0));
            double i1y = m*i1x + b;

            //y=60
            //y=mx+b
            //60=mx+b
            //(60-b)/m=x
            double i2x = (60 - b) / m;
            double i2y = 60;

            //x=64.4
            //y=mx+b
            //y=m*64.4+b
            double i3x = 64.4;
            double i3y = m*i3x + b;

            if(i1x >=  64.4){
                return new Pose2d(i3x, i3y, angle);
            }else if(i1y >= 60){
                return new Pose2d(i2x, i2y, angle);
            }else{
                return new Pose2d(i1x, i1y, angle);
            }
        }
    }


}

class Mineral{
    public double x;
    public double y;
    public double z;
    public String type;
    public Recognition obj;

    public Mineral(Recognition rec, double x, double y, double z){
        this.obj = rec;
        this.type = rec.getLabel();
        this.x = x;
        this.y = y;
        this.z = z;
    }
}