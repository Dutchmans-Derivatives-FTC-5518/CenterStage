package org.firstinspires.ftc.teamcode;

//import the necessary packages for instantiating Servo
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

public abstract class Ramp extends LinearOpMode{
    private double rampCurPosition; // attribute of ramp denoting its current position
    private boolean pixelLoaded; // attribute denoting if the ramp has a pixel

    public static final double UP = 0;  // static variable denoting ramp up
    public static final double STORE = 0.37; // static variable denoting ramp stored
    public static final double DOWN = 0.39; // static variable denoting ramp down
    public static final int STORE_WITH_PIXEL = 9; // static variable denoting ramp stored with pixel
//TODO: need to define what store with pixel is/ how we can change it
    // add static variables to denote up, down and store positions to pass on the SRV_R.setPosition(RampDownPos)

    // create necessary variables to control Ramp
    Servo SRV_R = hardwareMap.get(Servo.class, "ramp_srv");


    // instantiation of the class
    public Ramp() {
        rampCurPosition = STORE; //set position to store
        pixelLoaded = false; //set pixel loaded to false
    }

    // method to set the attribute that denotes if a pixel was loaded or unloaded = true/false
    public void setPixelLoaded(boolean pixelLoaded) {
        this.pixelLoaded = pixelLoaded;
    }

    //method to move the ramp up
    public void moveRampUp() {
        // Write code to move the Ramp up ex. SRV_R.setPosition(RampUpPos); //110 degrees
        SRV_R.setPosition(UP);
        rampCurPosition = UP;
    }

    public void moveRampDown() {
        // Write code to move the Ramp down ...
        // you can add check if pixel loaded to not move down
        if(!pixelLoaded){
            SRV_R.setPosition(DOWN);
            rampCurPosition = DOWN;
        }
    }

    public void storeRamp() {
        // Write code to store ramp
        // check if pixel loaded then set static variable appropriately
        //TODO: need to define what pixel loaded is/ how we can change it
        SRV_R.setPosition(STORE);
        if (pixelLoaded) {
            rampCurPosition = STORE_WITH_PIXEL;
        } else {
            rampCurPosition = STORE;
        }
    }

    // returns is ramp is up
    public boolean isRampUp() {
        return rampCurPosition == UP;
    }

    //returns if ramp is down
    public boolean isRampDown() {
        return rampCurPosition == DOWN;
    }

    //returns if ramp is stored
    public boolean isRampStored() {
        return rampCurPosition == STORE;
    }

    //returns true if ramp is stopred with pixel
    public boolean isRampStoredWithPixel() {
        return rampCurPosition == STORE_WITH_PIXEL;
    }
}
