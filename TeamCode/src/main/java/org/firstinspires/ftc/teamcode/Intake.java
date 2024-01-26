package org.firstinspires.ftc.teamcode;

// Import the necessary packages for instantiating Motor/Hardware Map
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake{
    // TODO: Check if these values such as power and time are correct for the intake
    private boolean deployed = false; // Attribute of if the intake mechanism has been dropped down at the start of the match
    private double deployPower = -0.5; // Attribute for how much power needed to deploy
    private long deployTime = 250; // Attribute for how much time needed to deploy
    private double intakePower = 0.1; // Power for how much the intake needs to spin pieces in
    private double outtakePower = -0.1; // Power for how much the intake needs to spin pieces out
    private boolean pixelLoaded = false; // Attribute denoting if there is a pixel thru the intake

    // TODO: Figure out a way to find out if a pixel has been intaken by the robot
    // Create necessary variables to set motor for intake
    DcMotor MTR_I;

    Telemetry telemetry;

    // Instantiation of the class
    public Intake(HardwareMap hardwareMap, Telemetry iTelemetry) {
        // Take the passed in value of telemetry and assign to class variables.
        telemetry = iTelemetry;

        // Get the motors/servos ready.
        MTR_I = hardwareMap.dcMotor.get("intake_mtr"); //create intake motor object
    }

    // Method to set the attribute that denotes if a pixel was loaded or unloaded = true/false
    public void setPixelLoaded(boolean pixelLoaded) {
        this.pixelLoaded = pixelLoaded;
    }

    // Method to drop the intake from the storage/start position.
    public void deployIntake() {
        // Write code to deploy the intake
        MTR_I.setPower(deployPower); // Motor is sent power values

        try {
            Thread.sleep(deployTime); // Motor sleeps
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        stopIntake();
    }

    // TODO: Test intake and outtake and see if they work in the main method (when button held)
    public void intakePixel() {
        // Write code to intake the pixel
        MTR_I.setPower(intakePower); // Motor is sent power values
        pixelLoaded = true; // Pixel is loaded
    }

    public void outtakePixel() {
        // Write code to intake the pixel
        MTR_I.setPower(outtakePower); // Motor is sent power values
        pixelLoaded = false; // Pixel is not loaded

		// TODO: The intake and outake might get a little confusing here. remember, the only
		// way to unload a pixel is to move the ramp to up position. Honestly, I don't think 
		// the intake needs to track the pixel since it just passes it to the ramp.
    }
    public void stopIntake() {
        // Write code to stop the intake
        MTR_I.setPower(0); // Motor is sent power values
    }

    // Returns if the intake is deployed
    public boolean isDeployed() {
        return deployed;
    }

    // Returns if pixel is loaded
    public boolean isPixelLoaded() {
        return pixelLoaded;
    }

    // Method returns ?...
    public void getTelemetryData() {
        //telemetry.addData("Ramp Position: ", SRV_R.getPosition());
    }
}