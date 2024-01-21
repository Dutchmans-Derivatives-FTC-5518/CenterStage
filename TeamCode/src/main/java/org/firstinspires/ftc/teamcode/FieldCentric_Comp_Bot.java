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
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class FieldCentric_Comp_Bot extends LinearOpMode{
    private int selection = 0;
    private double botHeading;
    DcMotorEx leftArm = (DcMotorEx) hardwareMap.dcMotor.get("left_arm");   // These varbiales do not match wiki documentation. Update.
    DcMotorEx rightArm = (DcMotorEx) hardwareMap.dcMotor.get("right_arm"); // See above.
    DcMotor MTR_LF = hardwareMap.dcMotor.get("left_front_mtr");
    DcMotor MTR_LB = hardwareMap.dcMotor.get("left_back_mtr");
    DcMotor MTR_RF = hardwareMap.dcMotor.get("right_front_mtr");
    DcMotor MTR_RB = hardwareMap.dcMotor.get("right_back_mtr");
    DcMotor MTR_I = hardwareMap.dcMotor.get("intake_mtr");
    IMU imu = hardwareMap.get(IMU.class, "imu");
    Servo SRV_R = hardwareMap.get(Servo.class, "ramp_srv");

//**********************************************************************************
//  Comment form Mr. Fisher & Mr. Nair
// New hubs are horizontal and 180 degrees off from each other. VERIFY THIS CODE and update comments.
// Remove this comment when checked.
//**********************************************************************************
  
    //Since our Exp. Hub is rotated and placed vertically, we have to configure the orientation on bot
    IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
            RevHubOrientationOnRobot.LogoFacingDirection.UP,
            RevHubOrientationOnRobot.UsbFacingDirection.LEFT));

    @Override
    public void runOpMode() throws InterruptedException {
        boolean initialized = false;
        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive()){
            //Initialize Robot
            if (!initialized){
                MTR_LF.setDirection(DcMotor.Direction.REVERSE);
                MTR_LB.setDirection(DcMotor.Direction.REVERSE);
                imu.initialize(parameters);
                SRV_R.setDirection(Servo.Direction.REVERSE);
                initialize();
                initialized = true;
            }

            drive();

            if(gamepad1.dpad_up){
                pickup();
            }
            if(gamepad1.dpad_down){
                storage();
            }

            //---------------------Gamepad 2 Controls/Arm Movement----------------------
            //Hotkeys (Automation)
            if (gamepad2.y)
                selection = 1;
            if (gamepad2.b)
                selection = 2;
            if (gamepad2.x)
                selection = 3;
            if (gamepad2.a)
                selection = 4;

            // Show the elapsed game time and wheel power.
            //Useful telemetry data incase needed for testing and to find heading of robot
            telemetry.addData("Left Front: ", MTR_LF.getPower());
            telemetry.addData("Left Back: ", MTR_LB.getPower());
            telemetry.addData("Right Front: ", MTR_RF.getPower());
            telemetry.addData("Right Back: ", MTR_RB.getPower());
            telemetry.addData("Heading: ", ((int)Math.toDegrees(botHeading)) + " degrees");
            telemetry.update();
        }
    }
    public void drive(){
        //---------------------Gamepad 1 Controls/Drivetrain Movement----------------------

        double y = -gamepad1.left_stick_y; //Reversed Value
        double x = gamepad1.left_stick_x * 1.7 ; //The double value on the left is a sensitivity setting (change when needed)
        double rx = gamepad1.right_stick_x; //Rotational Value

        //Find the first angle (Yaw) to get the robot heading
        botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        //Translate to robot heading from field heading for motor values
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        //Denominator is the largest motor power
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        MTR_LF.setPower(frontLeftPower);
        MTR_LB.setPower(backLeftPower);
        MTR_RF.setPower(frontRightPower);
        MTR_RB.setPower(backRightPower);
    }

    public void armMovement(int selection){
        //---------------------Gamepad 3 Controls/Arm Movement----------------------
        //Need code from aarush's laptop for this?
        System.out.println("filler");
    }

    public void initialize(){
        //---------------------Start of Match----------------------
        SRV_R.setPosition(0.37); //110 degrees
        MTR_I.setPower(-0.5);
        sleep(500);    // This might need to be a little longer...
        MTR_I.setPower(0);
    }


    public void pickup(){
        //---------------------Gamepad 1 Controls/Intake Movement----------------------
        SRV_R.setPosition(.39);//115 degrees
        sleep(1000);
        MTR_I.setPower(1);
    }


    public void storage(){
        //---------------------Gamepad 1 Controls/Ramp Movement----------------------
        MTR_I.setPower(0);
        SRV_R.setPosition(0.37); //110 degrees
    }
}

