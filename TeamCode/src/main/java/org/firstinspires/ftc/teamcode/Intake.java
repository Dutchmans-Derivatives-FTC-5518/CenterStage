package org.firstinspires.ftc.teamcode;

//import the necessary packages for instantiating Motor/Hardware Map
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;


public class Intake{
    private boolean deployed; // attribute of if the intake mechanism has been dropped down at the start of the match
    private double deployPower; //attribute for how much power needed to deploy
    private long deployTime; //attribute for how much time needed to deploy
    private double intakePower; //power for how much the intake needs to spin pieces in
    private double outtakePower; //power for how much the intake needs to spin pieces out
    private boolean pixelLoaded; // attribute denoting if there is a pixel thru the intake
    //TODO: Figure out a way to find out if a pixel has been intaken by the robot
    private boolean intakeDown; //attribute if the intake is down


    // create necessary variables to set motor for intake
    DcMotor MTR_I;


    // instantiation of the class
    public Intake() {
        //TODO: Check if these values such as power and time are correct for the intake
        deployed = false; //set deployed to true
        deployPower = -0.3;
        deployTime = 500;
        intakePower = 0.3;
        outtakePower = -0.3;
        pixelLoaded = false; //set pixel loaded to false
        intakeDown = false; // sets the intake down to false
        MTR_I = hardwareMap.dcMotor.get("intake_mtr"); //create intake motor object
    }

    // method to set the attribute that denotes if a pixel was loaded or unloaded = true/false
    public void setPixelLoaded(boolean pixelLoaded) {
        this.pixelLoaded = pixelLoaded;
    }

    //method to move the ramp up
    public void deployIntake() {
        // Write code to move the deploy the intake
        MTR_I.setPower(deployPower); //motor is sent power values
        try {
            Thread.sleep(deployTime); //motor sleeps for 1/2 a second
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        intakeDown = true; //intake is successfully down
    }

    //TODO: Test intake and outtake and see if they work in the main method (when button held)
    public void intakePixel() {
        // Write code to intake the pixel
        MTR_I.setPower(intakePower); //motor is sent power values
        pixelLoaded = true; //pixel is loaded
    }

    public void outtakePixel(){
        // Write code to intake the pixel
        MTR_I.setPower(outtakePower); //motor is sent power values
        pixelLoaded = false; //pixel is not loaded
    }

    // returns is intake is deployed
    public boolean isDeployed() {
        return deployed;
    }

    //returns if pixel is loaded
    public boolean isPixelLoaded() {
        return pixelLoaded;
    }

    //returns if intake is down
    public boolean isIntakeDown() {
        return intakeDown;
    }

}
