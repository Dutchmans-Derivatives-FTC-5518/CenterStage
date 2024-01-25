package org.firstinspires.ftc.teamcode;

//import the necessary packages for instantiating Servo/Hardware Map
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Ramp{
    private double rampCurPosition; // attribute of ramp denoting its current position
    private boolean pixelLoaded; // attribute denoting if the ramp has a pixel

    public static final double UP = 1;  // static variable denoting ramp up
    public static final double STORE = 0; // static variable denoting ramp stored
    public static final double DOWN = -1;
    public final double angUp= 0;  // static variable denoting ramp up
    public final double angStore = -0.37; // static variable denoting ramp stored
    public final double angDown = -0.39; // static variable denoting ramp down
    public static final int STORE_WITH_PIXEL = 9; // static variable denoting ramp stored with pixel
//TODO: need to define what store with pixel is/ how we can change it
    // add static variables to denote up, down and store positions to pass on the SRV_R.setPosition(RampDownPos)

    // create necessary variables to control Ramp/telemetry
    Servo SRV_R;
    Telemetry telemetry;

    // instantiation of the class
    public Ramp(HardwareMap hardwareMap) {
        rampCurPosition = STORE; //set position to store
        pixelLoaded = false; //set pixel loaded to false
        SRV_R = hardwareMap.get(Servo.class, "ramp_srv"); //Create servo object
        SRV_R.setDirection(Servo.Direction.REVERSE);
    }

    // method to set the attribute that denotes if a pixel was loaded or unloaded = true/false
    public void setPixelLoaded(boolean pixelLoaded) {
        this.pixelLoaded = pixelLoaded;
    }

    //method to move the ramp up
    public void moveRampUp() {
        // Write code to move the Ramp up ex. SRV_R.setPosition(RampUpPos); //110 degrees
        SRV_R.setPosition(angUp);
        rampCurPosition = UP;
    }

    public void moveRampDown() {
        // Write code to move the Ramp down ...
        // you can add check if pixel loaded to not move down
        if(!isPixelLoaded()){
            SRV_R.setPosition(angDown);
            rampCurPosition = DOWN;
        }
    }

    public void storeRamp() {
        // Write code to store ramp
        // check if pixel loaded then set static variable appropriately
        //TODO: need to define what pixel loaded is/ how we can change it
        SRV_R.setPosition(angStore);
        if (pixelLoaded) {
            rampCurPosition = STORE_WITH_PIXEL;
        } else {
            rampCurPosition = STORE;
        }
    }
    // returns if pixel is loaded
    public boolean isPixelLoaded() {
        return pixelLoaded;
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

    //returns true if ramp is stored with pixel
    public boolean isRampStoredWithPixel() {
        return rampCurPosition == STORE_WITH_PIXEL;
    }
    public void getTelemetryData() {
        telemetry.addData("Left Front: ", SRV_R.getPosition());
    }
}
