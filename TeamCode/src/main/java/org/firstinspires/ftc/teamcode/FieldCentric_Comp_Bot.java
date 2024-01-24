package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class FieldCentric_Comp_Bot extends LinearOpMode{
    private int selection = 0;
    private boolean initialized = false;
    DcMotorEx MTR_VSL = (DcMotorEx) hardwareMap.dcMotor.get("left_viper_mtr");
    DcMotorEx MTR_VSR = (DcMotorEx) hardwareMap.dcMotor.get("right_viper_mtr");
    Drivetrain myDriveTrain;
    Ramp myRamp;
    Intake myIntake;

/*
    TODO:
        1. Implement arm height code (WIP)
*/

    //@Override
    public FieldCentric_Comp_Bot(){
        myDriveTrain = new Drivetrain();
        myRamp = new Ramp();
        myIntake = new Intake();
    }
    public void runOpMode() throws InterruptedException {
        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive()){
            if (!initialized){
            FieldCentric_Comp_Bot myRobot = new FieldCentric_Comp_Bot();
            myIntake.deployIntake();
            initialized = true;
            }
            myDriveTrain.drive();
            if (gamepad1.left_trigger != 0){
                myRamp.moveRampDown();
                myIntake.intakePixel();
            }
            if (gamepad1.right_trigger != 0){
                myRamp.moveRampUp();
                myIntake.outtakePixel();
            }
            myDriveTrain.getTelemetryData();

/*
            //---------------------Gamepad 2 Controls/Arm Movement----------------------
            // Hotkeys (Automation)
            if (gamepad2.y)
                selection = 1;
            if (gamepad2.b)
                selection = 2;
            if (gamepad2.x)
                selection = 3;
            if (gamepad2.a)
                selection = 4;

            // Show the elapsed game time and wheel power.
            // Useful telemetry data in case needed for testing and to find heading of robot

            telemetry.update();
 */
        }
    }


/*
    public void armMovement(int selection) {

        //---------------------Gamepad 3 Controls/Arm Movement----------------------
        // TODO: Need code from Aarush's laptop for this?
        System.out.println("filler");
    }

    public void initialize(){

        //---------------------Start of Match----------------------
        SRV_R.setPosition(RampStorePos); //110 degrees
        MTR_I.setPower(-0.5);
        sleep(1500);    // TODO: Check the timing of this once robot is running. Maybe turn this into a global.
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
        SRV_R.setPosition(0.37); //110 degrees  // TODO: This should be a global.
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