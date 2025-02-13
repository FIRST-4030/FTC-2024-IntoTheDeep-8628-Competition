package org.firstinspires.ftc.teamcode.OpModes;

import static java.util.OptionalDouble.empty;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
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

import java.util.Optional;
import java.util.OptionalDouble;

@Config
@TeleOp(name = "FieldCentricMecanumTeleOp")
public class FieldCentricMecanumTeleOp extends LinearOpMode {
// good arm max    public static int armMaxPosition = 3270;
    public static class Arm {
        public int slideLowArmMaxPosition = 3641;
        public int slideHighArmMaxPosition = 4150;
        public int slideHighThreshold = 3200;
        public int armMaxPosition = slideHighArmMaxPosition;
        public int armRotationSpeed = 30; // in ticks per game loop
        public int armBoundingBoxEnforcedPosition = 2600;
        public int armHighChamber = 1357;
        public int armHighBucketPosition = 4150;
    }

    public static class Slide {
        public int slideMaxHorizontalPosition = 1564;
        public int slideMaxVerticalPosition = 3260;
        public int slideMovementSpeed = 15; // in ticks per game loop
        public int slideHighBucketPosition = 3260;
        public int slideBeginsExtendingForHighBucket = 1500;
    }

    public static class Wrist {
        public int wristBeginsFlippingForHighBucketArm = 3200;
        public int wristBeginsFlippingForHighBucketSlide = 2100;
        public double wristHighBucketDeliverPosition = 0.8;
    }

    public static class Hang {
        public int hangArmSubmersiblePosition = 4620;
        public int hangSlideSubmersiblePosition = 415;
        public double hangWristSubmersiblePosition = 0.39 ;
    }

    public static double clawIncrement = 1.0 / 300.0;
    public static double clawMax = 0.88;
    public static double clawMin = 0.04;
    public static double clawOpen = 0.04;
    public static double clawClose = 0.88;
    public static double wristMax = 0.8567;
    public static double wristMin = 0.05;

    public static double wristRotationStraight = 0.5;// TODO change this to the correct value

    public static Arm ARM = new Arm();
    public static Slide SLIDE = new Slide();
    public static Wrist WRIST = new Wrist();
    public static Hang HANG = new Hang();

    @Override
    public void runOpMode() throws InterruptedException {
        // Put constants here
        Optional<Double> initialAngle = Optional.empty();

        int armTargetPosition = 10;
        int armMinPosition = 10;
        int slideTargetPosition = 10;
        int slideMinPosition = 10;
        double clawTargetPosition = 0.88;
        double wristIncrement = 1.0/300.0;
        double wristRotationMax = 1.0;
        double wristRotationMin = 0.0;

        double wristRotationSpeed = 1.0/100.0;
        double wristRotationTargetPosition = wristRotationStraight;
        double wristStraightUp = 0.35;
        double wristTargetPosition = wristStraightUp;

        int pickupSlide = 1564;
        int pickupArm = 676;
        double pickupWrist = 0.1233;

        int scanArm = 1525;
        int scanSlide = 1564;
        double scanWrist = 0.05;

        double slowSpeed = 0.5;
        double superSlowSpeed = 0.25;

        double wristSubmersible = 0.2;

        double turnPower;
        double lastTurnPower = 0;
        Limelight3A limelight;
        // Declare our motors
        // Make sure your ID's match your configuration

        limelight = (Limelight3A) hardwareMap.get("limelight");
        limelight.setPollRateHz(100);
        limelight.start();
        limelight.pipelineSwitch(0);

        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("leftFront");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("leftBack");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("rightFront");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("rightBack");

        DcMotor slide = hardwareMap.dcMotor.get("slide");
        DcMotor arm = hardwareMap.dcMotor.get("arm");

        Servo claw = hardwareMap.servo.get("claw");
        Servo wrist = hardwareMap.servo.get("wrist");
        Servo wristRotation = hardwareMap.servo.get("wristRotation");

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
        wrist.setPosition(wristTargetPosition);
//        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setPower(1.0);
//        if(reverse) {arm.setDirection(DcMotorSimple.Direction.REVERSE);}
//        arm.setTargetPosition(0);
        arm.setTargetPosition(arm.getCurrentPosition());
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        tickLimit = maxVal;

//        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setPower(1.0);
//        if(reverse) {arm.setDirection(DcMotorSimple.Direction.REVERSE);}
        slide.setTargetPosition(slideMinPosition);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        tickLimit = maxVal;
        claw.setPosition(clawClose);

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
                // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        if (opModeInInit()) {
            telemetry.addData("Compiled on:", BuildConfig.COMPILATION_DATE);
            telemetry.update();
        }

        waitForStart();

        if (isStopRequested()) return;
//        InitializeArmAndSlide.initializeArmAndSlide(telemetry, claw, wrist, slide, arm, slideTouchSensor, armTouchSensor);
        while (opModeIsActive()) {
            double y = gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = -gamepad1.left_stick_x;
            double rx = -gamepad1.right_stick_x;
            int targetAngle;
            targetAngle = -100;   // -100 is a bogus value to allow if-else logic to work
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
                double minPowerToMoveRobot = 0.2;
                double angle = yawPitchRollAngles.getYaw();
                double error = targetAngle - angle;
                if (error > 180) {
                    error -= 360;
                }
                if (error < -180) {
                    error += 360;
                }
                turnPower = error/70.0;  // 70 is a value to force a low robot turn rate
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
                // Forces robot to move at a "slow" speed
                y *= slowSpeed;
                x *= slowSpeed;
                rx *= slowSpeed;
            }
            if (gamepad1.left_bumper){
                // Forces robot to move at a "super slow" speed
                y *= superSlowSpeed;
                x *= superSlowSpeed;
                rx *= superSlowSpeed;
            }

            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            if (gamepad1.options) {
                // Reset robot orientation
                imu.resetYaw();
            }
            if (gamepad1.back) {
                // Bring arm, slide, and claw back to there original position
                InitializeArmAndSlide.initializeArmAndSlide(telemetry, wristRotation, claw, wrist, slide, arm, slideTouchSensor, armTouchSensor);
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

            //bounding box
            if(slideTargetPosition < ARM.slideHighThreshold){
                ARM.armMaxPosition = ARM.slideLowArmMaxPosition;
            } else {
                ARM.armMaxPosition = ARM.slideHighArmMaxPosition;
            }

            if (gamepad2.left_stick_y != 0){
                // Rotate arm. If arm rotates to its max angle then allow
                // slide to move to no more than its max horizontal extension
                armTargetPosition += Math.floor(-gamepad2.left_stick_y*ARM.armRotationSpeed);
                armTargetPosition = Math.min(Math.max(armMinPosition,armTargetPosition),ARM.armMaxPosition);
                if (arm.getCurrentPosition() < ARM.armBoundingBoxEnforcedPosition){
                    slideTargetPosition = Math.min(Math.max(slideMinPosition,slideTargetPosition), SLIDE.slideMaxHorizontalPosition);
                    slide.setTargetPosition(slideTargetPosition);
                }
                arm.setTargetPosition(armTargetPosition);
            }
            if (gamepad2.right_stick_y != 0){
                // Move slide. If arm has rotated to its max angle then allow
                // slide to move no more than its max vertical extension.
                // Otherwise, limit slide extension to no more than its max horizontal setting.
                slideTargetPosition += Math.floor(-gamepad2.right_stick_y*SLIDE.slideMovementSpeed);
                if (armTargetPosition > ARM.armMaxPosition){
                    armTargetPosition = ARM.armMaxPosition;
                    arm.setTargetPosition(armTargetPosition);
                }
                if (arm.getCurrentPosition() > ARM.armBoundingBoxEnforcedPosition){
                    slideTargetPosition = Math.min(Math.max(slideMinPosition,slideTargetPosition), SLIDE.slideMaxVerticalPosition);
                } else {
                    slideTargetPosition = Math.min(Math.max(slideMinPosition,slideTargetPosition), SLIDE.slideMaxHorizontalPosition);
                }
                slide.setTargetPosition(slideTargetPosition);
            }
            if (gamepad1.dpad_down){
                if (!initialAngle.isPresent()){
                    initialAngle = Optional.of(getImuAngle(imu).getYaw(AngleUnit.DEGREES));
                }
                double currentAngle = getImuAngle(imu).getYaw(AngleUnit.DEGREES);
                double error = initialAngle.get()-currentAngle;
                if (error > 180){
                    error -= 180;
                } else if (error < -180) {
                    error += 180;
                }
                double limelightTurnPower = error/45;

                armTargetPosition = scanArm;
                arm.setTargetPosition(armTargetPosition);
                slideTargetPosition = scanSlide;
                slide.setTargetPosition(slideTargetPosition);
                wristTargetPosition = scanWrist;
                wrist.setPosition(wristTargetPosition);
                claw.setPosition(clawOpen);
                LLResult result = limelight.getLatestResult();
                double tx;
                double ty;
                if (result != null && result.isValid()) {
                    tx = result.getTx();
                    ty = result.getTy();
                    telemetry.addData("tx", tx);
                    telemetry.addData("ty", ty);

//                     if (tx >= 3){
//                         // goes left
//                         telemetry.addLine("going left");
//                        backLeftMotor.setPower(0.3);
//                        backRightMotor.setPower(-0.3);
//                        frontLeftMotor.setPower(-0.35);
//                        frontRightMotor.setPower(0.35);
//                    } else if (tx <= -3){
//                         telemetry.addLine("going right");
//                        backLeftMotor.setPower(-0.3);
//                        backRightMotor.setPower(0.3);
//                        frontLeftMotor.setPower(0.35);
//                        frontRightMotor.setPower(-0.35);
//                    } else if (ty >= 3){
//                        backLeftMotor.setPower(-0.3);
//                        backRightMotor.setPower(-0.3);
//                        frontLeftMotor.setPower(-0.35);
//                        frontRightMotor.setPower(-0.35);
//                    } else if (ty <= -3){
//                        backLeftMotor.setPower(0.3);
//                        backRightMotor.setPower(0.3);
//                        frontLeftMotor.setPower(0.35);
//                        frontRightMotor.setPower(0.35);
//                    }
                    if (Math.abs(tx) > 3 || Math.abs(ty) > 3){
                        move (tx, -ty, frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor, limelightTurnPower);
                    } else {
                        armTargetPosition = pickupArm;
                        arm.setTargetPosition(armTargetPosition);
                        slideTargetPosition = pickupSlide;
                        slide.setTargetPosition(slideTargetPosition);
                        wristTargetPosition = pickupWrist;
                        wrist.setPosition(wristTargetPosition);
                        backLeftMotor.setPower(0);
                        backRightMotor.setPower(0);
                        frontLeftMotor.setPower(0);
                        frontRightMotor.setPower(0);
                        sleep(1000);
                        claw.setPosition(clawClose);
                        sleep(300);
                    }
                }

            } else {
                initialAngle = Optional.empty();
            }

            if (gamepad1.dpad_right || gamepad2.dpad_right){
                wristRotationTargetPosition += wristRotationSpeed;
                wristRotationTargetPosition = Math.min(wristRotationMax,wristRotationTargetPosition);
                wristRotation.setPosition(wristRotationTargetPosition);
            } else if (gamepad1.dpad_left || gamepad2.dpad_left){
                wristRotationTargetPosition -= wristRotationSpeed;
                wristRotationTargetPosition = Math.max(wristRotationMin,wristRotationTargetPosition);
                wristRotation.setPosition(wristRotationTargetPosition);
            } else if (gamepad1.dpad_up){
                wristRotationTargetPosition = wristRotationStraight;
                wristRotation.setPosition(wristRotationTargetPosition);
            }
            if (gamepad2.dpad_up){
                // Move wrist up
                wristTargetPosition += wristIncrement;
                wristTargetPosition = Math.min (wristMax,wristTargetPosition);
                wrist.setPosition(wristTargetPosition);
            } else if (gamepad2.dpad_down){
                // Move wrist down
                wristTargetPosition -= wristIncrement;
                wristTargetPosition = Math.max (wristMin,wristTargetPosition);
                wrist.setPosition(wristTargetPosition);
            }
            if (gamepad2.right_trigger > 0.2){
                // Incrementally close claw
                clawTargetPosition += clawIncrement;
                clawTargetPosition = Math.min (clawMax,clawTargetPosition);
                claw.setPosition(clawTargetPosition);
            } else if (gamepad2.left_trigger > 0.2){
                // Incrementally open claw
                clawTargetPosition -= clawIncrement;
                clawTargetPosition = Math.max (clawMin,clawTargetPosition);
                claw.setPosition(clawTargetPosition);
            }
            if (gamepad1.right_trigger > 0.2 || gamepad2.right_bumper){
                // Close claw
                clawTargetPosition = clawClose;
                claw.setPosition(clawTargetPosition);
            } else if (gamepad1.left_trigger > 0.2 || gamepad2.left_bumper){
                // Open claw
                clawTargetPosition = clawOpen;
                claw.setPosition(clawTargetPosition);
            }
            if (gamepad2.y){
                // For sample side, begin to move arm to its highest angle
                armTargetPosition = ARM.armHighBucketPosition;
                arm.setTargetPosition(armTargetPosition);
                wristRotationTargetPosition = wristRotationStraight;
                wristRotation.setPosition(wristRotationTargetPosition);
                int currentArmPosition = arm.getCurrentPosition();
                // Keep wrist straight up until arm gets to a specific angle
                if (currentArmPosition < WRIST.wristBeginsFlippingForHighBucketArm) {
                    wristTargetPosition = wristStraightUp;
                    wrist.setPosition(wristTargetPosition);
                }
                // Move slide once arm crosses a specific angle
                if (currentArmPosition > SLIDE.slideBeginsExtendingForHighBucket){
                    slideTargetPosition = SLIDE.slideHighBucketPosition;
                    slide.setTargetPosition(slideTargetPosition);
                    // Move wrist to fully open when slide and arm are fully extended
                    if (currentArmPosition > WRIST.wristBeginsFlippingForHighBucketArm && slide.getCurrentPosition() > WRIST.wristBeginsFlippingForHighBucketSlide) {
                        wristTargetPosition = WRIST.wristHighBucketDeliverPosition;
                        wrist.setPosition(wristTargetPosition);
                    }
                }
            }
            if (gamepad2.a){
                // For sample side, retract arm to pick up from floor
                armTargetPosition = 756;
                arm.setTargetPosition(armTargetPosition);
                wristTargetPosition = 0.1;
                wrist.setPosition(wristTargetPosition);
                wristRotationTargetPosition = wristRotationStraight;
                wristRotation.setPosition(wristRotationTargetPosition);

                slideTargetPosition = 724;
                slide.setTargetPosition(slideTargetPosition);
            }
            if (gamepad2.b){
                // For specimen side, move all to position for pickup
                armTargetPosition = 356;
                arm.setTargetPosition(armTargetPosition);
                wristTargetPosition = 0.43;
                wrist.setPosition(wristTargetPosition);

                slideTargetPosition = 46;
                slide.setTargetPosition(slideTargetPosition);
                wristRotationTargetPosition = wristRotationStraight;
                wristRotation.setPosition(wristRotationTargetPosition);
                //claw.setPosition(clawOpen);
            }
            if (gamepad2.x){
                // For specimen side, move all to place specimen on high chamber
                armTargetPosition = ARM.armHighChamber;
                arm.setTargetPosition(armTargetPosition);
                wristTargetPosition = 0.5667;
                wrist.setPosition(wristTargetPosition);
                if (arm.getCurrentPosition() > 750) {
                    slideTargetPosition = 1114;
                    slide.setTargetPosition(slideTargetPosition);
                }
                claw.setPosition(clawClose);
            }
            if (gamepad2.back) {
                // Hang robot from the low rung
                armTargetPosition = HANG.hangArmSubmersiblePosition;
                slideTargetPosition = HANG.hangSlideSubmersiblePosition;
                wristTargetPosition = HANG.hangWristSubmersiblePosition;
                arm.setTargetPosition(armTargetPosition);
                slide.setTargetPosition(slideTargetPosition);
                wrist.setPosition(wristTargetPosition);
            }

            telemetry.addData("armPosition", arm.getCurrentPosition());
            telemetry.addData("slidePosition", slide.getCurrentPosition());
            telemetry.addData("armTargetPosition", armTargetPosition);
            telemetry.addData("slideTargetPosition", slideTargetPosition);
            telemetry.addData("wristTargetPosition", wristTargetPosition);
            telemetry.addData("clawTargetPosition", clawTargetPosition);
            telemetry.addData("wristRotationTargetPosition", wristRotationTargetPosition);

            telemetry.update();
        }

    }
    public void move (double x /* degrees to the left */, double y, DcMotor frontLeftMotor, DcMotor backLeftMotor,
                      DcMotor frontRightMotor, DcMotor backRightMotor, double turnPower){
        double minPower = 0.3;
        double powerLeft = 0;
        if (x > 3){
            powerLeft = Math.max (minPower, x/20);
        } else if (x < 3) {
            powerLeft = Math.max (-minPower, x/20);
        }
        double powerForward = 0;
        if (y > 3){
            powerForward = Math.max (minPower, y/20);
        } else if (y < 3){
            powerForward = Math.max (-minPower, y/20);
        }
        telemetry.addData("powerLeft", powerLeft);
        telemetry.addData("powerForward", powerForward);

        double frontLeftPower = powerForward-powerLeft+turnPower;
        double backLeftPower = powerForward+powerLeft+turnPower;
        double frontRightPower = powerForward+powerLeft-turnPower;
        double backRightPower = powerForward-powerLeft-turnPower;

        telemetry.addData("frontLeftPower", frontLeftPower);
        telemetry.addData("backLeftPower", backLeftPower);
        telemetry.addData("frontRightPower", frontRightPower);
        telemetry.addData("backRightPower", backRightPower);

        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);

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