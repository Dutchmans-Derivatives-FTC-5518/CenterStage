package org.firstinspires.ftc.teamcode;

//import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

//import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class FieldCentric_Comp_Bot extends LinearOpMode{
    Drivetrain myDriveTrain;
    Ramp myRamp;
    Intake myIntake;
    Gripper myGripper;
    private final boolean isDebugMode = true;  // Used to deactivate DEBUG related code when testing isn't being used.

    //@Override

    public void runOpMode() throws InterruptedException { // TODO: We should do something proper with this thrown exception to make sure we handle things cleanly.
        myDriveTrain = new Drivetrain(this);
        myRamp = new Ramp(this);
        myIntake = new Intake(this);
        myGripper = new Gripper(this);

        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive())
        {
            // Pre-match set up start positions
            myIntake.deployIntake(); // Deploy the intake from the storage position to start match.
            myGripper.guardGripper(); // Put gripper into the guard position to start match.
            myRamp.moveRampStore(); // Place ramp in the store position for start match.
            myGripper.moveSlideDown(); // TODO: this isn't the right function here. This should be updated after all the new code for the viper slides is written.

            //**************************************************************************************
            // ============== GamePad 1 Controls ====================================================
            myDriveTrain.drive();
            if (gamepad1.left_trigger != 0){   // TODO: All of these should be thought about and made sure they are working as intended.
                myRamp.moveRampDown();  // Move ramp to the floor
                sleep(250);
                myIntake.intakePixel(); // Start intake roller
                // TODO: Should we disable the drivetrain at this point so we don't drag the ramp on the ground?
            }
            if (gamepad1.left_trigger == 0 || gamepad1.right_trigger == 0){
                myIntake.stopIntake(); // stop Intake
                sleep(250); // Pause for a moment
                myRamp.moveRampStore(); // Move the ramp to STORE
                // TODO: Re-enable the drivetrain?
            }
            if (gamepad1.right_trigger != 0){
                myRamp.moveRampSet(); // Move ramp to set position to make sure Pixel in in the proper location.
                sleep(500); // Pause for a moment  //TODO: This might be too short. Test it by watching the function on the robot.
                myRamp.moveRampUp();  // Drop the pixel into the rest.
            }
            if (gamepad1.a) {  // Cross Symbol
                myIntake.outtakePixel(); // Used to clear a jam, not sure this will work or be needed.
            }

            //**************************************************************************************
            //============== GamePad 2 Controls ====================================================
            if (gamepad2.y) { // Triangle Symbol
                myGripper.moveSlideDown();
                myGripper.openGripper();
            }
            if (gamepad2.b) { // Circle Symbol
                myGripper.moveSlideLow();
            }
            if (gamepad2.x) { // Square Symbol
                myGripper.moveSlideMiddle();
            }
            if (gamepad2.a) { // Cross Symbol
                myGripper.moveSlideHigh();
            }
            if (gamepad2.left_trigger != 0) {
                myGripper.openGripper();
            }
            if (gamepad2.right_trigger != 0) {
                myGripper.closeGripper();
            }
            // Useful telemetry data in case needed for testing and to find heading of robot
            myDriveTrain.getTelemetryData();
            myRamp.getTelemetryData();
            myGripper.getTelemetryData();
            telemetry.update();

            //**************************************************************************************
            //============== Debug Code and Tools ==================================================
            if (isDebugMode) {  // Check to see if debug mode is on. Set in class header.
                if (gamepad1.b) { // Circle
                    myRamp.debugRamp(0.4);
                    sleep(2500);
                    myRamp.debugRamp(0.60);
                    sleep(2500);
                    myRamp.debugRamp(0.3);
                    sleep(2500);
                    myRamp.debugRamp(0.22);

                    // Move grippers to the closed position
                    myGripper.debugGripper();
                }
            }
        }
    }

    // TODO: We should get rod of the code below if not needed.
/*
    public void armMovement(int selection) {
        //---------------------Gamepad 3 Controls/Arm Movement----------------------
        System.out.println("filler");
    }
    public void initialize(){
        //---------------------Start of Match----------------------
        SRV_R.setPosition(RampStorePos); //110 degrees
        MTR_I.setPower(-0.5);
        sleep(1500);
        MTR_I.setPower(0);
        isRampDown = false;
        isRampStore = true;
        isRampUp = false;
    }
    public void pickup(){
        //---------------------Gamepad 1 Controls/Intake Movement----------------------
        SRV_R.setPosition(RampDownPos); //115 degrees
        sleep(1000);
        MTR_I.setPower(1);
        isRampDown = true;
        isRampStore = false;
        isRampUp = false;
    }
    public void storage(){
        //---------------------Gamepad 1 Controls/Ramp Movement----------------------
        MTR_I.setPower(0);
        SRV_R.setPosition(0.37); //110 degrees
        isRampDown = false;
        isRampStore = true;
        isRampUp = false;
    }
    public void up(){
        //---------------------Gamepad 1 Controls/Ramp Movement----------------------
        SRV_R.setPosition(RampUpPos); //110 degrees
        isRampDown = false;
        isRampStore = false;
        isRampUp = true;
    }
 */
}