package org.firstinspires.ftc.teamcode;

//import the necessary packages for instantiating Motor/Hardware Map
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
public class Drone{
    private boolean launched; // attribute of if the airplane has been launched
    private double angleUnlock; //power for unlocking slider
    Servo Launcher;

    FieldCentric_Comp_Bot bot;

    // instantiation of the class
    public Drone(FieldCentric_Comp_Bot iBot) {
        // Take the passed in value of telemetry and assign to class variables.
        bot = iBot;

        launched = false;
        angleUnlock = 1;
        Launcher = bot.hardwareMap.servo.get("SRV_3"); //create intake motor object
    }

    public void launchDrone() {
        Launcher.setPosition(angleUnlock);
        launched = true;
    }
}