package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class FieldCentric_Comp_Bot extends LinearOpMode{
    Drivetrain myDriveTrain = null;
    Ramp myRamp;
    Intake myIntake;
    Gripper myGripper;
    Drone myDrone;

    //@Override

    public void runOpMode() throws InterruptedException {
        myDriveTrain = new Drivetrain(this);
        myRamp = new Ramp(this);
        myIntake = new Intake(this);
        myGripper = new Gripper(this);
        myDrone = new Drone(this);

        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive()){
            //**************************************************************************************
            // ---------------------Gamepad 1 Controls ---------------------------------------------
            myDriveTrain.drive();

            if (gamepad1.left_trigger != 0){
                if (!myRamp.isRampDown()) {
                    myRamp.moveRampDown();
                }
                myIntake.intakePixel();
            }
            else if (gamepad1.right_trigger != 0){
                myIntake.outtakePixel();
            }
            else if (gamepad1.left_trigger == 0 || gamepad1.right_trigger == 0){
                myIntake.stopIntake();
            }

            if (gamepad1.dpad_down){
                myRamp.moveRampDown();
            }
            else if (gamepad1.dpad_left){
                myRamp.moveRampStore();
            }
            else if (gamepad1.dpad_up){
                myRamp.moveRampStore();
                sleep(1000);
                myRamp.moveRampUp();
            }
            if(gamepad1.left_trigger > 0.5 && gamepad2.right_trigger > 0.5){myDrone.launchDrone();}

            //**************************************************************************************
            // ---------------------Gamepad 2 Controls ---------------------------------------------
            // Hotkeys (Automation to raise slide up)
            if (gamepad2.y) {
                myGripper.moveSlideDown();
                myGripper.openGripper();
            }
            else if (gamepad2.b){
                myGripper.moveSlideLow();
            }
            else if (gamepad2.x){
                myGripper.moveSlideMiddle();
            }
            else if (gamepad2.a){
                myGripper.moveSlideHigh();
            }

            // Hotkeys (to change gripper position)
            if (gamepad2.left_trigger != 0) {
                myGripper.openGripper();
            }
            else if (gamepad2.right_trigger != 0) {
                myGripper.closeGripper();
            }

            //**************************************************************************************
            //--------------------- DEBUG Code --------------------------------------------
            if (gamepad1.b){
                myGripper.debugSlide();
//                myRamp.debugRamp(0.48);
//                sleep(2500);
//                myRamp.debugRamp(0.15);
//                sleep(2500);
//                myRamp.debugRamp(0.1);
            }

            //**************************************************************************************
            //--------------------- TELEMETRY Code --------------------------------------------
            // Useful telemetry data in case needed for testing and to find heading of robot
            myDriveTrain.getTelemetryData();
            myRamp.getTelemetryData();
            myGripper.getTelemetryData();
            telemetry.update();
        }
    }
}