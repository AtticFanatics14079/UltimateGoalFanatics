package org.firstinspires.ftc.teamcode.JONSKETCH.DriveObjectLibrary;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Arrays;


public class HardwareThread extends Thread {

    ElapsedTime time;
    private ValueStorage vals;
    double[][] hardwareVals; //See hardwareValues in ValueStorage for each value.
    boolean[] changedParts; //See hardwareValues in ValueStorage for each value.
    double[] runVals; //See hardwareValues in ValueStorage for each value.
    public Configuration config;
    //public ConfigurationRR config;
    private volatile boolean stop;
    private boolean setTime = false;
    public double voltMult = 1, lastTime = 0;

    public HardwareThread(ValueStorage valStorage, HardwareMap hwMap, Configuration configuration){
        this.vals = valStorage;
        config = configuration;
        config.Configure(hwMap, valStorage);
        vals.setup(config.hardware.size());
        double[] temp = new double[1];
        runVals = new double[config.hardware.size()];
        hardwareVals = new double[config.hardware.size()][temp.length];
        changedParts = new boolean[config.hardware.size()];
        Arrays.fill(changedParts, false);
        Arrays.fill(runVals, 0.0);
        Arrays.fill(temp, 0.0);
        Arrays.fill(hardwareVals, temp);
        vals.setup(config.hardware.size());
        //voltMult = 13.0/config.voltSense.getVoltage();
        config.setBulkCachingManual();
    }

    public void run(){
        while(!setTime && !stop){}
        while(!stop) {
            if(time.milliseconds() - lastTime >= 5) {
                lastTime = time.milliseconds();
                vals.time(true, time.milliseconds());
                readHardware(vals.changedParts(false, null));
                vals.hardware(true, hardwareVals);
                runHardware(vals.runValues(false, null), vals.changedParts(false, null));
            }
        }
        for(DriveObject d : config.hardware) {
            d.endAllThreads();
            if(d.getType() == DriveObject.type.DcMotorImplEx) {
                d.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                d.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            vals.clear();
        }
    }

    public void startTime(ElapsedTime time){
        try {
            sleep((long) 2.5);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        this.time = time;
        setTime = true;
    }

    private void readHardware(boolean[] changedParts){

        config.clearBulkCache();

        this.changedParts = changedParts.clone();

        for(int i = 0; i < this.changedParts.length; i++) {
            if (this.changedParts[i])
                hardwareVals[i] = config.hardware.get(i).getHardware();
        }
    }

    private void runHardware(double[] Values, boolean[] changedParts) {
        //Same values for desiredParts as above's changedParts.

        for(int i = 0; i < Values.length; i++) runVals[i] = Values[i];
        this.changedParts = changedParts.clone();

        for(int i = 0; i < this.changedParts.length; i++) {
            if (this.changedParts[i])
                config.hardware.get(i).setHardware(runVals[i]);
        }
    }

    public void Stop(){
        stop = true;
    }
}
