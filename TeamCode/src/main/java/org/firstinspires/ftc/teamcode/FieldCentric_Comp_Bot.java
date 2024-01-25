package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class FieldCentric_Comp_Bot extends LinearOpMode{
    Drivetrain myDriveTrain = null;
    Ramp myRamp;
    Intake myIntake;
    Gripper myGripper;

    //@Override


    public void runOpMode() throws InterruptedException {
        myDriveTrain = new Drivetrain(this.hardwareMap, this.gamepad1, this.telemetry);
        myRamp = new Ramp(this.hardwareMap);
        myIntake = new Intake(this.hardwareMap);
        myGripper = new Gripper(this.hardwareMap);

        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive()){
            myDriveTrain.drive();
            if (gamepad1.left_trigger != 0){
                myRamp.moveRampDown();
                myIntake.intakePixel();
            }
            if (gamepad1.left_trigger == 0 || gamepad1.right_trigger == 0){
                myIntake.stopIntake();
            }
            if (gamepad1.right_trigger != 0){
                myRamp.moveRampUp();
            }
            if (gamepad1.a){ myIntake.outtakePixel(); }
            myDriveTrain.getTelemetryData();

            //---------------------Gamepad 2 Controls/Arm Movement----------------------
            // Hotkeys (Automation to raise slide up)
            if (gamepad2.y) {
                myGripper.moveSlideDown();
                myGripper.openGripper();
            }
            if (gamepad2.b){myGripper.moveSlideLow();}
            if (gamepad2.x){myGripper.moveSlideMiddle();}
            if (gamepad2.a) {myGripper.moveSlideHigh();}
            // Hotkeys (to change gripper position)
            if (gamepad2.left_trigger != 0) {myGripper.openGripper();}
            if (gamepad2.right_trigger != 0) {myGripper.closeGripper();}
            // Show the elapsed game time and wheel power.
            // Useful telemetry data in case needed for testing and to find heading of robot
            telemetry.update();
        }
    }
/*
    public void armMovement(int selection) {

        //---------------------Gamepad 3 Controls/Arm Movement----------------------
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