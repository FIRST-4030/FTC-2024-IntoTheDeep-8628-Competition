package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class FieldCentricMecanumTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Put constants here
        int armMaxPosition = 100000;
        int armBucketPosition = 150;
        int armTargetPosition = 0;
        int armMinPosition = 0;
        int armRotationSpeed = 10;
        int slideMaxPosition = 100000;
        int slideBucketPosition = 150;
        int slideTargetPosition = 0;
        int slideMinPosition = 0;
        int slideMovementSpeed = 10;
        double clawTargetPosition = 0.5;
        double clawSpeed = 1.0/100.0;
        double clawMax = 1.0;
        double clawMin = 0.46;
        double wristTargetPosition = 0.5;
        double wristSpeed = 1.0/300.0;
        double wristMax = 0.89;
        double wristMin = 0.03;
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("leftFront");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("leftBack");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("rightFront");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("rightBack");
//        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("leftFront");
//        DcMotor backLeftMotor = hardwareMap.dcMotor.get("leftBack");
//        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRight");
//        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRight");
        DcMotor slide = hardwareMap.dcMotor.get("slide");
        DcMotor arm = hardwareMap.dcMotor.get("pivot");
        Servo claw = hardwareMap.servo.get("claw");
        Servo wrist = hardwareMap.servo.get("wrist");


        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        arm.setDirection(DcMotorSimple.Direction.REVERSE);
        slide.setDirection(DcMotorSimple.Direction.REVERSE);


        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setPower(1.0);
//        if(reverse) {arm.setDirection(DcMotorSimple.Direction.REVERSE);}
        arm.setTargetPosition(0);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        tickLimit = maxVal;

        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setPower(1.0);
//        if(reverse) {arm.setDirection(DcMotorSimple.Direction.REVERSE);}
        slide.setTargetPosition(0);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        tickLimit = maxVal;

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = -gamepad1.left_stick_x;
            double rx = -gamepad1.right_stick_x;


            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            if (gamepad1.options) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            if (gamepad2.left_stick_y != 0){
                armTargetPosition += Math.floor(-gamepad2.left_stick_y*armRotationSpeed);
                arm.setTargetPosition(armTargetPosition);
                armTargetPosition = Math.min(Math.max(armMinPosition,armTargetPosition),armMaxPosition);
            }
            if (gamepad2.right_stick_y != 0){
                slideTargetPosition += Math.floor(-gamepad2.right_stick_y*slideMovementSpeed);
                slideTargetPosition = Math.min(Math.max(slideMinPosition,slideTargetPosition),slideMaxPosition);
                slide.setTargetPosition(slideTargetPosition);
            }
            if (gamepad2.dpad_up){
                wristTargetPosition += wristSpeed;
                wristTargetPosition = Math.min (wristMax,wristTargetPosition);
            } else if (gamepad2.dpad_down){
                wristTargetPosition -= wristSpeed;
                wristTargetPosition = Math.max (wristMin,wristTargetPosition);
            }
            wrist.setPosition(wristTargetPosition);
            if (gamepad2.right_bumper){
                clawTargetPosition += clawSpeed;
                clawTargetPosition = Math.min (clawMax,clawTargetPosition);
            } else if (gamepad2.left_bumper){
                clawTargetPosition -= clawSpeed;
                clawTargetPosition = Math.max (clawMin,clawTargetPosition);
            }
            claw.setPosition(clawTargetPosition);

            telemetry.addData("armPosition", arm.getCurrentPosition());
            telemetry.addData("slidePosition", slide.getCurrentPosition());
            telemetry.addData("armTargetPosition", armTargetPosition);
            telemetry.addData("slideTargetPosition", slideTargetPosition);
            telemetry.addData("wristTargetPosition", wristTargetPosition);
            telemetry.addData("clawTargetPosition", clawTargetPosition);

            telemetry.update();
        }
    }
}