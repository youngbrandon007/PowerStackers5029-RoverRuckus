package org.firstinspires.ftc.teamcode.PSRobotLibs;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

public class PSTelemetry {

    private Telemetry telemetry;
    HashMap<String, String> telemetryData;
    private ArrayList<String> disabled;

    public PSTelemetry(Telemetry telemetry){
        this.telemetry = telemetry;
        clear();
        clearDisabled();
    }

    public void enable(String name){
        if(disabled.contains(name)) disabled.remove(name);
    }

    public void disable(String name){
        disabled.add(name);
    }

    public void clear(){
        telemetryData = new HashMap<String, String>();
    }

    public void clearDisabled(){
        disabled = new ArrayList<String>();
    }

    public void update(){
        updateNoClear();
        clear();
    }

    public void updateNoClear(){
        if(telemetryData.size() == 0){
            return;
        }
        for (Map.Entry<String, String> entry : telemetryData.entrySet()) {
            String name = entry.getKey();
            Object value = entry.getValue();
            String section = null;
            section = (name.contains(".")) ? name.split(".")[0] : name;
            if(!disabled.contains(section)){
                telemetry.addData(name, value);
            }
        }
    }

    public void updateTelemtry(){
        telemetry.update();
    }

    public void updateAll(){
        update();
        telemetry.update();
    }

    public void updateAdd(){
        updateNoClear();
        telemetry.update();
    }



    public void add(String name, String data){
        telemetryData.put(name, String.valueOf(data));
    }
    public void add(String name, int data){
        telemetryData.put(name, String.valueOf(data));
    }
    public void add(String name, double data){
        telemetryData.put(name, String.valueOf(data));
    }
    public void add(String name, boolean data){
        telemetryData.put(name, String.valueOf(data));
    }
    public void add(String name, float data){
        telemetryData.put(name, String.valueOf(data));
    }
    public void add(String name, Object data){
        telemetryData.put(name, String.valueOf(data));
    }
}
