package frc.robot2025.utils;

import java.util.ArrayList;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class UXTrim {
    //Table to keep all the trims
    final static String tableName = "/TrimTable";
    static NetworkTable TrimTable = NetworkTableInstance.getDefault().getTable(tableName);

    //All the trimable values we are tracking
    static ArrayList<UXTrim> trims = new ArrayList<UXTrim>();

    //instance vars
    final DoubleSubscriber trimSub;
    final DoubleTopic d_topic;

    //data, and optional accessor, optional change callback
    double trim;
    DoubleSupplier dataSupplier;
    Supplier<Boolean> changeCallback;

    public UXTrim(String name){
        this(name, 0.0);
    }

    public UXTrim(String name, DoubleSupplier dataSupplier){
        this(name, 0.0, dataSupplier);
    }

    public UXTrim(String name, double trim) {     
        changeCallback = null;
        dataSupplier = null;
                
        //create the topic, and subscribe
        d_topic = TrimTable.getDoubleTopic(name);
        trimSub = d_topic.subscribe(trim);
        // get the persisted value, if there is one
        this.trim = trimSub.get();
        // publish & persist
        d_topic.publish().set(this.trim);        
        d_topic.setPersistent(true);
      
        trims.add(this);
    }

    public UXTrim(String name, double trim, DoubleSupplier dataSupplier) {
        this(name, trim);
        this.dataSupplier = dataSupplier;
    }

    public double getValue() {
        return dataSupplier.getAsDouble() + trim;
    }

    public double getValue(double value){
        return value + trim;
    }

    public UXTrim addDoubleSupplier(DoubleSupplier dataSupplier) {
        this.dataSupplier = dataSupplier;
        return this;
    }

    public UXTrim addChangeCallback(Supplier<Boolean> callback) {
        changeCallback = callback;
        return this;
    }

    //Hook this into the robot loop somewhere 
    static public void periodic() {
        //monitor subs for new trim values
        for (UXTrim uxTrim : trims) {
            double newval = uxTrim.trimSub.get();
            if (newval != uxTrim.trim) {
                uxTrim.trim = newval;
                if (uxTrim.changeCallback != null) 
                    uxTrim.changeCallback.get(); // don't care about returned bool
            }
        }
    }

}
