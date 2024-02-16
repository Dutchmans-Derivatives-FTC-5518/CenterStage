package org.firstinspires.ftc.teamcode;

//import the necessary packages for instantiating Motor/Hardware Map
import com.qualcomm.robotcore.hardware.DcMotor;

public class Intake{
    private boolean deployed; // attribute of if the intake mechanism has been dropped down at the start of the match
    private double deployPower; //attribute for how much power needed to deploy
    private long deployTime; //attribute for how much time needed to deploy
    private double intakePower; //power for how much the intake needs to spin pieces in
    private double outtakePower; //power for how much the intake needs to spin pieces out
    private boolean pixelLoaded; // attribute denoting if there is a pixel thru the intake
    //TODO: Figure out a way to find out if a pixel has been intaken by the robot

    // create necessary variables to set motor for intake
    DcMotor MTR_I;

    FieldCentric_Comp_Bot bot;

    // instantiation of the class
    public Intake(FieldCentric_Comp_Bot iBot) {
        // Take the passed in value of telemetry and assign to class variables.
        bot = iBot;

        // TODO: Check if these values such as power and time are correct for the intake
        deployed = false; // set deployed to true
        deployPower = -0.5;
        deployTime = 250;  // Quarter second
        intakePower = .5;
        outtakePower = -.55;
        pixelLoaded = false; // set pixel loaded to false
        MTR_I = bot.hardwareMap.dcMotor.get("intake_mtr"); //create intake motor object
        deployIntake();
    }

    // method to set the attribute that denotes if a pixel was loaded or unloaded = true/false
    public void setPixelLoaded(boolean pixelLoaded) {
        this.pixelLoaded = pixelLoaded;
    }

    // method to drop the intake from the storage/start position.
    //Deploy intake after running mode, not after initialization
    public void deployIntake() {
        // Write code to deploy the intake
        MTR_I.setPower(deployPower); // motor is sent power values
        try {
            Thread.sleep(deployTime); // motor sleeps
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        MTR_I.setPower(0);
    }

    // TODO: Test intake and outtake and see if they work in the main method
    public void intakePixel() {
        // Write code to intake the pixel
        MTR_I.setPower(intakePower); //motor is sent power values
        pixelLoaded = true; //pixel is loaded
    }

    public void outtakePixel(){
        // Write code to intake the pixel
        MTR_I.setPower(outtakePower); // motor is sent power values
        pixelLoaded = false; // pixel is not loaded

		// TODO: The intake and outake might get a little confusing here. remember, the only
		// way to unload a pixel is to move the ramp to up position. Honestly, I don't think 
		// the intake needs to track the pixel since it just passes it to the ramp.
    }
    public void stopIntake() {
        // Stop the intake from spinning
        MTR_I.setPower(0); //motor is sent power values
    }

    // returns is intake is deployed
    public boolean isDeployed() {
        return deployed;
    }

    // returns if pixel is loaded
    public boolean isPixelLoaded() {
        return pixelLoaded;
    }
}