package org.firstinspires.ftc.teamcode;

//import the necessary packages for instantiating Motor
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

//import com.qualcomm.robotcore.hardware.HardwareMap;
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.Gamepad;
//import com.qualcomm.robotcore.hardware.IMU;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Gripper{
    private Servo SRV_LG, SRV_RG;
    private DcMotor MTR_LVS, MTR_RVS;
    private int gripperCurrPosition;
    private static final double MTR_LVS_PW = 0.5;
    private static final double MTR_RVS_PW = 0.35;
    private int open = 1;
    private int closed = -1;
    private int guard = 0;
    private double dGripperOpen = 0.0;
    private double dGripperGuard = 0.12;
    private double dGripperClosed = 0.19;
    FieldCentric_Comp_Bot bot;

    // TODO: Check encoders

    public Gripper(HardwareMap hwMap) {
        SRV_LG = hwMap.get(Servo.class, "left_grip_srv");
        SRV_RG = hwMap.get(Servo.class, "right_grip_srv");
        MTR_LVS = hwMap.get(DcMotor.class, "left_viper_mtr");
        MTR_RVS = hwMap.get(DcMotor.class, "right_viper_mtr");
        //TODO: May have to reverse the motor direction to spin other way
        // Set encoder to 0 ticks
        MTR_LVS.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MTR_RVS.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MTR_LVS.setDirection(DcMotor.Direction.REVERSE);
        guardGripper();
    }
    /*public Gripper(HardwareMap hardwareMap, Telemetry iTelemetry) {
        // Take the passed in value of telemetry and assign to class variables.
        telemetry = iTelemetry;

        // Set up motors
        //TODO: May have to reverse the motor direction to spin other way
        SRV_LG = hardwareMap.get(Servo.class, "left_grip_srv");
        SRV_RG = hardwareMap.get(Servo.class, "right_grip_srv");
        // Set servo of left gripper to run the opposite direction.
=======
        // Restart motor encoder
        moveSlideDown();
        // Open Grippers
        SRV_LG.setDirection(Servo.Direction.REVERSE);
        openGripper();
        gripperCurrPosition = open;
        open = 1;
        closed = -1;
        guard = 0;
    }

    public void openGripper() {
        SRV_LG.setPosition(dOpen);
        SRV_RG.setPosition(dOpen);
        gripperCurrPosition = open;
    }

    public void guardGripper() {
        SRV_LG.setPosition(dGuard);
        SRV_RG.setPosition(dGuard);
        gripperCurrPosition = guard;
        //TODO: Must test these values and see if they are the right angle
    }

    public void closeGripper() {
        SRV_LG.setPosition(dClosed);
        SRV_RG.setPosition(dClosed);
        gripperCurrPosition = closed;
    }

    public void moveSlideDown() {
        MTR_LVS.setTargetPosition(0);
        MTR_RVS.setTargetPosition(0);
        guardGripper();
        MTR_LVS.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        MTR_RVS.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        powerArm();
    }

    public void moveSlideLow() {
        MTR_LVS.setTargetPosition(500);
        MTR_RVS.setTargetPosition(500);
        MTR_LVS.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        MTR_RVS.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        powerArm();
    }

    public void moveSlideMiddle() {
        MTR_LVS.setTargetPosition(1000);
        MTR_RVS.setTargetPosition(1000);
        MTR_LVS.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        MTR_RVS.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        powerArm();
    }

    public void moveSlideHigh() {
        MTR_LVS.setTargetPosition(3000);
        MTR_RVS.setTargetPosition(3000);
        MTR_LVS.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        MTR_RVS.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        powerArm();
    }

    public void powerArm() {
        MTR_LVS.setPower(MTR_LVS_PW);
        MTR_RVS.setPower(MTR_RVS_PW);
    }
    // Method returns gripper servo positions
    public void getTelemetryData() {
        bot.telemetry.addData("SRV_RG Position: ", SRV_RG.getPosition());
        bot.telemetry.addData("SRV_LG Position: ", SRV_LG.getPosition());
        bot.telemetry.addData("MTR_LVS Position: ", MTR_LVS.getCurrentPosition());
        bot.telemetry.addData("MTR_RVS Position: ", MTR_RVS.getCurrentPosition());
    }
    public void debugSlide() {
        MTR_LVS.setTargetPosition(-3600);
        MTR_LVS.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        MTR_LVS.setPower(MTR_LVS_PW);

        MTR_RVS.setTargetPosition(3800);
        MTR_RVS.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        MTR_RVS.setPower(MTR_RVS_PW);
    }
    public void debugGripper(double angle) {
        SRV_LG.setPosition(angle);
        SRV_RG.setPosition(angle);
    }
}