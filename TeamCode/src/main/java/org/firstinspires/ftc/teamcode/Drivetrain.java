package org.firstinspires.ftc.teamcode;

//import the necessary packages for instantiating Motor
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Drivetrain{
    private double y; //value of y on joystick
    private double x; //value of x on joystick
    private double rx; //rotation value
    private double rotX; //rotational value of x
    private double rotY; //rotational value of y
    private double botHeading; //direction of bot
    //Values of power
    private double leftFrontPower;
    private double leftBackPower;
    private double rightFrontPower;
    private double rightBackPower;

    // Create necessary variables to control each 4 wheels and IMU
    DcMotor MTR_LF = null;
    DcMotor MTR_LB = null;
    DcMotor MTR_RF = null;
    DcMotor MTR_RB = null;
    IMU imu;
    Gamepad gamepad1 = null;
    Telemetry telemetry = null;

    // instantiation of the class
    public Drivetrain(HardwareMap hardwareMap, Gamepad iGamepad1, Telemetry iTelemetry) {

        // Take the passed in value of gamepad1 and assign to class instance of gamepad1
        gamepad1 = iGamepad1;
        telemetry = iTelemetry;

        MTR_LF = hardwareMap.dcMotor.get("left_front_mtr"); //instantiate 4 motors
        MTR_LB = hardwareMap.dcMotor.get("left_back_mtr");
        MTR_RF = hardwareMap.dcMotor.get("right_front_mtr");
        MTR_RB = hardwareMap.dcMotor.get("right_back_mtr");
        imu = hardwareMap.get(IMU.class, "imu");
        MTR_LF.setDirection(DcMotor.Direction.REVERSE);
        MTR_LB.setDirection(DcMotor.Direction.REVERSE);
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        imu.initialize(parameters);
        //TODO: Test the IMU parameters and see if it works
        leftFrontPower = 0.0;
        leftBackPower = 0.0;
        rightFrontPower = 0.0;
        rightBackPower = 0.0;
    }

    public void drive(){
        //TODO: Test these sensitivity values
        //---------------------Gamepad 1 Controls/Drivetrain Movement----------------------//
        y = -(gamepad1.left_stick_y); // Reversed Value
        x = gamepad1.left_stick_x * 1.7 ; // The double value on the left is a sensitivity setting (change when needed)
                                          // TODO: The constant in line above should be a global.
        rx = gamepad1.right_stick_x; // Rotational Value

        // Find the first angle (Yaw) to get the robot heading
        botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Translate to robot heading from field heading for motor values
        rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        // Denominator is the largest motor power
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        leftFrontPower = (rotY + rotX + rx) / denominator;
        leftBackPower = (rotY - rotX + rx) / denominator;
        rightFrontPower = (rotY - rotX - rx) / denominator;
        rightBackPower = (rotY + rotX - rx) / denominator;

        MTR_LF.setPower(leftFrontPower);
        MTR_LB.setPower(leftBackPower);
        MTR_RF.setPower(rightFrontPower);
        MTR_RB.setPower(rightBackPower);
    }

    // returns power to left front motor
    public double getLeftFrontPower() {
        return leftFrontPower;
    }

    //returns power to left back motor
    public double getLeftBackPower() {
        return leftBackPower;
    }

    //returns power to right front motor
    public double getRightFrontPower() {
        return rightFrontPower;
    }

    //returns power to right back motor
    public double getRightBackPower() {
        return rightBackPower;
    }
    public double getBotHeading() {return botHeading;}
    public void getTelemetryData() {
        telemetry.addData("Left Front: ", getLeftFrontPower());
        telemetry.addData("Left Back: ", getLeftBackPower());
        telemetry.addData("Right Front: ", getRightFrontPower());
        telemetry.addData("Right Back: ", getRightBackPower());
        telemetry.addData("Heading: ", ((int) Math.toDegrees(getBotHeading())) + " degrees");
        telemetry.update();
    }
}