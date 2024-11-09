package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.BuildConfig;

@Config
@TeleOp(name = "FieldCentricMecanumTeleOp")
public class FieldCentricMecanumTeleOp extends LinearOpMode {
// good arm max    public static int armMaxPosition = 3270;
public static int armMaxPosition = 3300; // arm max disabled
    public static int slideMaxHorizontalPosition = 1060;
    public static int slideMaxVerticalPosition = 2300;

    @Override
    public void runOpMode() throws InterruptedException {
        // Put constants here
        int armTargetPosition = 10;
        int armMinPosition = 10;
        int armRotationSpeed = 20;
        int armBoundingBoxEnforcedPosition = 2600;
        int slideTargetPosition = 10;
        int slideMinPosition = 10;
        int slideMovementSpeed = 10;
        double clawTargetPosition = 0.5;
        double clawSpeed = 1.0/30.0;
        double clawMax = 0.84;
        double clawMin = 0.25;
        double clawOpen = 0.25;
        double clawClose = 0.84;
        double wristTargetPosition = 0.5;
        double wristSpeed = 1.0/300.0;
        double wristMax = 0.95;
        double wristMin = 0.39;

        double wristStraightUp = 0.5;

        double slowSpeed = 0.5;
        double superSlowSpeed = 0.25;

        double wristSubmersible = 0.2;
        // driver assist high bucket arm, slide and wrist movement
        int slideHighBucketPosition = 2200;
        int armHighBucketPosition = 3270;
        int slideBeginsExtendingForHighBucket = 1500;
        int wristBeginsFlippingForHighBucketArm = 3200;
        int wristBeginsFlippingForHighBucketSlide = 2100;
        double wristHighBucketDeliverPosition = 0.95;

        double turnPower;
        double lastTurnPower = 0;
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
        TouchSensor armTouchSensor = hardwareMap.get(TouchSensor.class, "armTouchSensor");
        TouchSensor slideTouchSensor = hardwareMap.get(TouchSensor.class, "slideTouchSensor");


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
        InitializeArmAndSlide.initializeArmAndSlide(telemetry, claw, wrist, slide, arm, slideTouchSensor, armTouchSensor);
        while (opModeIsActive()) {
            double y = gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = -gamepad1.left_stick_x;
            double rx = -gamepad1.right_stick_x;
            int targetAngle;
            targetAngle = -100;
            if (gamepad1.a){
                targetAngle = 180;
            } else if (gamepad1.b) {
                targetAngle = -90;
            } else if (gamepad1.x) {
                targetAngle = 90;
            } else if (gamepad1.y) {
                targetAngle = 0;
            }
            if (targetAngle >= -90){
                double maxChange = 0.1;
                YawPitchRollAngles yawPitchRollAngles = getImuAngle(imu);
                AngularVelocity angularVelocity = getAngularVelocity(imu);
                double velocity = angularVelocity.zRotationRate;
                double minPowerToMoveRobot = 0.25;
                double angle = yawPitchRollAngles.getYaw();
                double error = targetAngle - angle;
                if (error > 180) {
                    error += -360;
                }
                if (error < -180) {
                    error += 360;
                }
                turnPower = error/70.0;
//                double addSomething = 0;
//                if (velocity == 0){
//                    addSomething = 0.01;
//                }
//                turnPower += 1/(velocity*100+addSomething);
                if (turnPower > 0){
                    turnPower = Math.max(minPowerToMoveRobot,turnPower);
                } else if (turnPower < 0){
                    turnPower = Math.min(-minPowerToMoveRobot,turnPower);
                }
                turnPower = Math.max (-1.0,turnPower);
                turnPower = Math.min (1.0, turnPower);
//                turnPower = Math.max(lastTurnPower-turnPower,-maxChange);
//                turnPower = Math.min(lastTurnPower-turnPower,maxChange);
                if (Math.abs(error) < 1){
                    turnPower = 0;
                }
                rx = turnPower;
                lastTurnPower = turnPower;
            } else {
                lastTurnPower = 0;
            }

            if (gamepad1.right_bumper){
                y *= slowSpeed;
                x *= slowSpeed;
                rx *= slowSpeed;
            }
            if (gamepad1.left_bumper){
                y *= superSlowSpeed;
                x *= superSlowSpeed;
                rx *= superSlowSpeed;
            }


            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            if (gamepad1.options) {
                imu.resetYaw();
            }
            if (gamepad1.back) {
                InitializeArmAndSlide.initializeArmAndSlide(telemetry, claw, wrist, slide, arm, slideTouchSensor, armTouchSensor);
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
                if (arm.getCurrentPosition() < armBoundingBoxEnforcedPosition){
                    slideTargetPosition = Math.min(Math.max(slideMinPosition,slideTargetPosition), slideMaxHorizontalPosition);
                    slide.setTargetPosition(slideTargetPosition);
                }
                arm.setTargetPosition(armTargetPosition);
            }
            if (gamepad2.right_stick_y != 0){
                slideTargetPosition += Math.floor(-gamepad2.right_stick_y*slideMovementSpeed);
                if (arm.getCurrentPosition() > armBoundingBoxEnforcedPosition){
                    slideTargetPosition = Math.min(Math.max(slideMinPosition,slideTargetPosition), slideMaxVerticalPosition);
                } else {
                    slideTargetPosition = Math.min(Math.max(slideMinPosition,slideTargetPosition), slideMaxHorizontalPosition);
                }
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
            if (gamepad2.right_trigger > 0.2){
                clawTargetPosition = clawClose;
                claw.setPosition(clawTargetPosition);
            } else if (gamepad2.left_trigger > 0.2){
                clawTargetPosition = clawOpen;
                claw.setPosition(clawTargetPosition);
            }
            if (gamepad2.y){
                armTargetPosition = armHighBucketPosition;
                arm.setTargetPosition(armTargetPosition);
                int currentArmPosition = arm.getCurrentPosition();
                if (currentArmPosition < wristBeginsFlippingForHighBucketArm) {
                    wristTargetPosition = wristStraightUp;
                    wrist.setPosition(wristTargetPosition);
                }
                if (currentArmPosition > slideBeginsExtendingForHighBucket){
                    slideTargetPosition = slideHighBucketPosition;
                    slide.setTargetPosition(slideTargetPosition);
                    if (currentArmPosition > wristBeginsFlippingForHighBucketArm && slide.getCurrentPosition() > wristBeginsFlippingForHighBucketSlide) {
                        wristTargetPosition = wristHighBucketDeliverPosition;
                        wrist.setPosition(wristTargetPosition);
                    }
                }
            }
            if (gamepad2.a){
                armTargetPosition = 520;
                arm.setTargetPosition(armTargetPosition);
                wristTargetPosition = 0.45;
                wrist.setPosition(wristTargetPosition);

                slideTargetPosition = 583;
                slide.setTargetPosition(slideTargetPosition);
            }
            if (gamepad2.b){
                armTargetPosition = 10;
                arm.setTargetPosition(armTargetPosition);
                wristTargetPosition = 0.7437;
                wrist.setPosition(wristTargetPosition);

                slideTargetPosition = 453;
                slide.setTargetPosition(slideTargetPosition);
                claw.setPosition(clawOpen);
            }
            if (gamepad2.x){
                armTargetPosition = 1500;
                arm.setTargetPosition(armTargetPosition);
                wristTargetPosition = 0.8567;
                wrist.setPosition(wristTargetPosition);

                slideTargetPosition = 827;
                slide.setTargetPosition(slideTargetPosition);
                claw.setPosition(clawClose);
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
    public YawPitchRollAngles getImuAngle(IMU imu){
        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
        return angles;
    }
    public AngularVelocity getAngularVelocity(IMU imu){
        AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
        return  angularVelocity;
    }
}