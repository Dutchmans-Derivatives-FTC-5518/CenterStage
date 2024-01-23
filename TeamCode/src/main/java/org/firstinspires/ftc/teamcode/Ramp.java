package org.firstinspires.ftc.teamcode;

//import the necessary packages for instantiating Servo


public class Ramp {
    private int rampCurPosition; // attribute of ramp denoting its current position
    private boolean pixelLoaded; // attribute denoting if the ramp has a pixel

    public static final int UP = 1;  // static variable denoting ramp up
    public static final int STORE = 0; // static variable denoting ramp stored
    public static final int DOWN = -1; // static variable denoting ramp down
    public static final int STORE_WITH_PIXEL = 9; // static variable denoting ramp stored with pixel

    // add static variables to denote up, down and store positions to pass on the SRV_R.setPosition(RampDownPos)

    // create necessary variables to control Ramp
    // Servo SRV_R = hardwareMap.get(Servo.class, "ramp_srv");


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
        rampCurPosition = UP;
    }

    public void moveRampDown() {
        // Write code to move the Ramp down ...
        // you can add check if pixel loaded to not move down
        rampCurPosition = DOWN;
    }

    public void storeRamp() {
        // Write code to store ramp
        // check if pixel loaded then set static variable appropriately
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
