package org.firstinspires.ftc.teamcode;

//import the necessary packages for instantiating Servo/Hardware Map
import com.qualcomm.robotcore.hardware.Servo;

//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
//import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class Ramp{
    private double rampCurPosition; // attribute of ramp denoting its current position
    private boolean pixelLoaded; // attribute denoting if the ramp has a pixel

    public static final double UP = 2;  // static variable denoting ramp up
    public static final double SET = 1;
    public static final double STORE = 0; // static variable denoting ramp stored
    public static final double DOWN = -1;
    public final double angUp = 0.6;  // static variable denoting ramp up
    public final double angStore = 0.4; // static variable denoting ramp stored
    public final double angDown = 0.20; // static variable denoting ramp down
    public static final int STORE_WITH_PIXEL = 9; // static variable denoting ramp stored with pixel

    // TODO: need to define what store with pixel is/ how we can change it
    // Create necessary variables to control Ramp/telemetry
    Servo SRV_R;
    // Variable to hold passed in telemetry variable.
    FieldCentric_Comp_Bot bot;

    // Instantiation of the class
    public Ramp(FieldCentric_Comp_Bot iBot) {
        // Take the passed in value of telemetry and assign to class variables.
        bot = iBot;

        rampCurPosition = STORE; // Set position to store
        pixelLoaded = false; // Set pixel loaded to false
        SRV_R = bot.hardwareMap.get(Servo.class, "ramp_srv"); //Create servo object
        moveRampStore();
    }

    // Method to set the attribute that denotes if a pixel was loaded or unloaded = true/false
    public void setPixelLoaded(boolean pixelLoaded) {
        this.pixelLoaded = pixelLoaded;
    }

    // Move the ramp to UP Position
    public void moveRampUp() {
        // Write code to move the Ramp up ex. SRV_R.setPosition(RampUpPos); //110 degrees
        SRV_R.setPosition(angUp);
        rampCurPosition = UP;
    }

    // Move the ramp to SET position
    /*
    public void moveRampSet() {
        // Write code to move the Ramp up ex. SRV_R.setPosition(RampUpPos); //110 degrees
        SRV_R.setPosition(angStore);
        rampCurPosition = SET;
    }
*/
    // Move the ramp to DOWN position
    public void moveRampDown() {
        // You can add check if pixel loaded to not move down
        SRV_R.setPosition(angDown);
        rampCurPosition = DOWN;
        //if(!isPixelLoaded()){}
    }
    // Method used to put the ramp into the STORE position and set the tracking variables.
    public void moveRampStore() {
        // Check if pixel loaded then set static variable appropriately
        // TODO: need to define what pixel loaded is/ how we can change it
        SRV_R.setPosition(angStore);
        if (pixelLoaded) {
            rampCurPosition = STORE_WITH_PIXEL;
        } else {
            rampCurPosition = STORE;
        }
    }

    // Method returns if pixel is loaded
    public boolean isPixelLoaded() {
        return pixelLoaded;
    }

    // Method returns is ramp is up
    public boolean isRampUp() {
        return rampCurPosition == UP;
    }

    // Method returns if ramp is down
    public boolean isRampDown() {
        return rampCurPosition == DOWN;
    }

    // Method returns if ramp is stored
    public boolean isRampStored() {
        return rampCurPosition == STORE;
    }

    // Method returns true if ramp is stored with pixel
    public boolean isRampStoredWithPixel() {
        return rampCurPosition == STORE_WITH_PIXEL;
    }

    // Method returns current Ramp Servo position
    public void getTelemetryData() {
        bot.telemetry.addData("Ramp Position: ", SRV_R.getPosition());
    }

    // Method used to easily test the ramps functionality. This function, if possible, should use
    // local methods to perform operations.
    public void debugRamp(double angle) {
        SRV_R.setPosition(angle);
    }
}

