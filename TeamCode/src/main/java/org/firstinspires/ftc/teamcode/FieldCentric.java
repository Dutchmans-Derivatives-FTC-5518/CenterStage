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
public class FieldCentric_Comp_Bot extends LinearOpMode {
    private int selection = 0;
    private double botHeading;
    //Configuration (FYI: motor initialization in class extension)
    //Servo forebar = hardwareMap.get(Servo.class, "forebar");
    //CRServo forebar = hardwareMap.get(CRServo.class, "forebar");


    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx leftArm = (DcMotorEx) hardwareMap.dcMotor.get("left_arm");
        DcMotorEx rightArm = (DcMotorEx) hardwareMap.dcMotor.get("right_arm");
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("left_front");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("left_back");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("right_front");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("right_back");
        //DcMotor armMotorR = hardwareMap.dcMotor.get("armMotorR");
        //DcMotor armMotorL = hardwareMap.dcMotor.get("armMotorL");
        IMU imu = hardwareMap.get(IMU.class, "imu");
        CRServo bucket = hardwareMap.get(CRServo.class, "bucket");
        //Reverse Left Motors
        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotor.Direction.REVERSE);
        //armMotorL.setDirection(DcMotorSimple.Direction.REVERSE);

        //Since our Exp. Hub is rotated and placed vertically, we have to configure the orientation on bot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            //---------------------Gamepad 1 Controls/Drivetrain Movement----------------------

            //Take analog
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

            //Set power values
            motorFrontLeft.setPower(frontLeftPower);
            motorBackLeft.setPower(backLeftPower);
            motorFrontRight.setPower(frontRightPower);
            motorBackRight.setPower(backRightPower);

            //---------------------Gamepad 2 Controls/Arm Movement----------------------

            //Forebar Control
            //forebar.scaleRange(35,350);

            /**
             * WIP
             */

            /*//Arm controls
            if (gamepad2.left_stick_y > 0.2) {
                armMotor.setPower(-.5);
                //armMotorL.setPower(.7);
                //armMotorR.setPower(.7);
            } else if (gamepad2.left_stick_y < -0.2) {
                armMotor.setPower(1);
                //armMotorL.setPower(-.5);
                //armMotorR.setPower(-.5);
            } else if (armMotor.getCurrentPosition()<800) {
                armMotor.setPower(0);
                //armMotorL.setPower(0);
                //armMotorR.setPower(0);
            } else {
                armMotor.setPower(.0055);
                //armMotorL.setPower(.0055);
                //armMotorR.setPower(.0055);
            }*/

            //Hotkeys (Automation)
            if (gamepad2.y)
                selection = 1;
            if (gamepad2.b)
                selection = 2;
            if (gamepad2.x)
                selection = 3;
            if (gamepad2.a)
                selection = 4;

            //armHeight(selection);

            //Bucket controls
            if (gamepad2.left_trigger > 0.2) {
                bucket.setPower(-1);

            } else if (gamepad2.right_trigger > 0.2) {
                bucket.setPower(1);

            } else {
                bucket.setPower(0);

            }

            // Show the elapsed game time and wheel power.
            //Useful telemetry data incase needed for testing and to find heading of robot
            telemetry.addData("Left Front: ", motorFrontLeft.getPower());
            telemetry.addData("Right Front: ", motorFrontRight.getPower());
            telemetry.addData("Left Back: ", motorBackLeft.getPower());
            telemetry.addData("Right Back: ", motorBackRight.getPower());
            telemetry.addData("Heading: ", ((int)Math.toDegrees(botHeading)) + " degrees");
            telemetry.update();
        }
    }

    /*
    public void armHeight(int s){
        if(s == 1)
        {
            armMotor.setTargetPosition(-3500);
            //armMotorL.setTargetPosition(-3500);
            //armMotorR.setTargetPosition(-3500);
            armMotor.setPower(-.5);
            //armMotorL.setPower(-.5);
            //armMotorR.setPower(-.5);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //armMotorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //armMotorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        if(s == 2)
        {
            armMotor.setTargetPosition(-2500);
            //armMotorL.setTargetPosition(-2500);
            //armMotorR.setTargetPosition(-2500);
            armMotor.setPower(-.5);
            //armMotorL.setPower(-.5);
            //armMotorR.setPower(-.5);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //armMotorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //armMotorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        if(s == 3)
        {
            armMotor.setTargetPosition(-1500);
            //armMotorL.setTargetPosition(-1500);
            //armMotorR.setTargetPosition(-1500);
            armMotor.setPower(-.5);
            //armMotorL.setPower(-.5);
            //armMotorR.setPower(-.5);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //armMotorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //armMotorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        if(s == 4)
        {
            armMotor.setTargetPosition(-300);
            //armMotorL.setTargetPosition(-300);
            //armMotorR.setTargetPosition(-300);
            armMotor.setPower(-.5);
            //armMotorL.setPower(-.5);
            //armMotorR.setPower(-.5);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //armMotorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //armMotorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        selection = 0;

    }
    */
}
