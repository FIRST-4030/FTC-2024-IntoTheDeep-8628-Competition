package org.firstinspires.ftc.teamcode.OpModes;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.BuildConfig;

@Config
@TeleOp(name = "FieldCentricMecanumTeleOp", group="X8628")
public class FieldCentricMecanumTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Put constants here
        int armMaxPosition = 3380;
        int armTargetPosition = 10;
        int armMinPosition = 10;
        int armRotationSpeed = 20;
        int slideMaxPosition = 2300;
        int slideTargetPosition = 10;
        int slideMinPosition = 10;
        int slideMovementSpeed = 10;
        double clawTargetPosition = 0.5;
        double clawSpeed = 1.0/30.0;
        double clawMax = 1.0;
        double clawMin = 0.46;
        double wristTargetPosition = 0.5;
        double wristSpeed = 1.0/300.0;
        double wristMax = 0.89;
        double wristMin = 0.03;

        double slowSpeed = 0.5;

        double wristSubmersible = 0.2;
        // driver assist high bucket arm, slide and wrist movement
        int slideHighBucketPosition = 2200;
        int armHighBucketPosition = 3380;
        int slideBeginsExtendingForHighBucket = 1500;
        int wristBeginsFlippingForHighBucket = 2600;
        double wristHighBucketDeliverPosition = 0.84;
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
        DcMotor arm = hardwareMap.dcMotor.get("arm");
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
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        if (opModeInInit()) {
            String compilationDate = BuildConfig.COMPILATION_DATE;
            telemetry.addData("Compiled on:", compilationDate);
            telemetry.update();
        }

        waitForStart();

        if (isStopRequested()) return;
        TouchSensor armTouchSensor = hardwareMap.get(TouchSensor.class, "armTouchSensor");
        TouchSensor slideTouchSensor = hardwareMap.get(TouchSensor.class, "slideTouchSensor");
        InitializeArmAndSlide.initializeArmAndSlide(telemetry, claw, wrist, slide, arm, slideTouchSensor, armTouchSensor);
        while (opModeIsActive()) {
            double y = gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = -gamepad1.left_stick_x;
            double rx = -gamepad1.right_stick_x;
            if (gamepad1.right_bumper){
                y *= slowSpeed;
                x *= slowSpeed;
                rx *= slowSpeed;
            }

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
                armTargetPosition = Math.min(Math.max(armMinPosition,armTargetPosition),armMaxPosition);
                arm.setTargetPosition(armTargetPosition);
            }
            if (gamepad2.right_stick_y != 0){
                slideTargetPosition += Math.floor(-gamepad2.right_stick_y*slideMovementSpeed);
                slideTargetPosition = Math.min(Math.max(slideMinPosition,slideTargetPosition),slideMaxPosition);
                slide.setTargetPosition(slideTargetPosition);
            }
            if (gamepad2.dpad_up){
                wristTargetPosition += wristSpeed;
                wristTargetPosition = Math.min (wristMax,wristTargetPosition);
                wrist.setPosition(wristTargetPosition);
            } else if (gamepad2.dpad_down){
                wristTargetPosition -= wristSpeed;
                wristTargetPosition = Math.max (wristMin,wristTargetPosition);
                wrist.setPosition(wristTargetPosition);
            }
            if (gamepad2.right_bumper){
                clawTargetPosition += clawSpeed;
                clawTargetPosition = Math.min (clawMax,clawTargetPosition);
                claw.setPosition(clawTargetPosition);
            } else if (gamepad2.left_bumper){
                clawTargetPosition -= clawSpeed;
                clawTargetPosition = Math.max (clawMin,clawTargetPosition);
                claw.setPosition(clawTargetPosition);
            }
            if (gamepad2.right_trigger > 0){
                wristTargetPosition = wristMin;
                wrist.setPosition(wristTargetPosition);
            } else if (gamepad2.left_trigger > 0){
                wristTargetPosition = wristMax;
                wrist.setPosition(wristTargetPosition);
            }
            if (gamepad2.y){
                armTargetPosition = armHighBucketPosition;
                arm.setTargetPosition(armTargetPosition);
                int currentArmPosition = arm.getCurrentPosition();
                if (currentArmPosition > slideBeginsExtendingForHighBucket){
                    slideTargetPosition = slideHighBucketPosition;
                    slide.setTargetPosition(slideTargetPosition);
                    if (currentArmPosition > wristBeginsFlippingForHighBucket) {
                        wristTargetPosition = wristHighBucketDeliverPosition;
                        wrist.setPosition(wristTargetPosition);
                    }
                }
            }
            if (gamepad2.a){
                armTargetPosition = armMinPosition;
                arm.setTargetPosition(armTargetPosition);

                if (arm.getCurrentPosition() > slideBeginsExtendingForHighBucket){
                    slideTargetPosition = slideMinPosition;
                    slide.setTargetPosition(slideTargetPosition);
                    wristTargetPosition = wristSubmersible;
                    wrist.setPosition(wristTargetPosition);
                }
            }

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