package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.ComputerVision;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Pose2dWrapper;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;

import java.util.Arrays;

@Config
@Autonomous(name = "RightAuto", group="8628")
public final class RightAuto extends LinearOpMode {

    public static double startX = 8;
    public static double startY = -62;
    public static double startHeading = -90;

    public static double highChamberDeliverWrist = 0.95;
    public static int highChamberDeliverArm = 2910;
    public static int highChamberDeliverSlide = 750;
    public static double highChamberPrepWrist = 0.95;
    public static int highChamberPrepArm = 2910;
    public static int highChamberPrepSlide = 289;
    public static double highChamberX = 0.0;
    public static double highChamberY = -33.0;
    public static double highChamberHeading = -90;
    public static int pickupArmPosition = 10;
    public static int pickupSlidePosition = 260;
    public static double pickupWristPosition = 0.7437;
    public static double pickupXPosition = 36.0;
    public static double pickupYPosition = -55.0;
    public static double pickupHeading = -90;
    double clawOpen = 0.25;
    double clawClosed = 0.84;

    ComputerVision vision;
    AprilTagPoseFtc[] aprilTagTranslations = new AprilTagPoseFtc[11];
    //InputHandler inputHandler;

    boolean inputComplete = false;
    Pose2d robotPose;

    ElapsedTime runtime = new ElapsedTime();
    ElapsedTime inputTimer = new ElapsedTime();
    int startDelay = 0;
    int i = 1; //used as an iterator for outputLog()

    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor slide = hardwareMap.dcMotor.get("slide");
        DcMotor arm = hardwareMap.dcMotor.get("arm");
        Servo claw = hardwareMap.servo.get("claw");
        Servo wrist = hardwareMap.servo.get("wrist");
        TouchSensor armTouchSensor = hardwareMap.get(TouchSensor.class, "armTouchSensor");
        TouchSensor slideTouchSensor = hardwareMap.get(TouchSensor.class, "slideTouchSensor");

        arm.setDirection(DcMotorSimple.Direction.REVERSE);
        slide.setDirection(DcMotorSimple.Direction.REVERSE);
        claw.setPosition(clawClosed);
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

        //runtime.reset();
        //inputHandler = InputAutoMapper.normal.autoMap(this);
        /*while (inputComplete == false) {


            if (inputHandler.up("D1:X")) {
                inputComplete = true;
                inputTimer.reset();
            }

        }
        telemetry.addData("-----Initialization-----", "");
        telemetry.addLine();
        telemetry.addData("-----Modifications-----", "");
        telemetry.addLine();
        telemetry.addData("Press X to finalize values", inputComplete);
        telemetry.update();


        vision = new ComputerVision(hardwareMap);
        while (vision.visionPortal.getCameraState() == OPENING_CAMERA_DEVICE) {}

        vision.setActiveCameraOne(); */

        VelConstraint baseVelConstraint = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(40.0),
                new AngularVelConstraint(Math.PI / 2)
        ));
        VelConstraint curveVelConstraint = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(15.0),
                new AngularVelConstraint(Math.PI / 2)
        ));
        AccelConstraint baseAccelConstraint = new ProfileAccelConstraint(-20.0, 50.0);


        Pose2dWrapper startPose = new Pose2dWrapper(startX, startY, Math.toRadians(startHeading));

        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose.toPose2d());

         InitializeArmAndSlide.initializeArmAndSlide(telemetry, claw, wrist, slide, arm, slideTouchSensor, armTouchSensor);

        waitForStart();
//                        .splineToConstantHeading(new Vector2d(-33.0, 30.00), Math.toRadians(-90.0), baseVelConstraint,baseAccelConstraint)


        Pose2d highChamberDeliverPose = new Pose2d(highChamberX, highChamberY, Math.toRadians(highChamberHeading));
        Pose2d lastPose = startPose.toPose2d();
        Pose2d thisPose = highChamberDeliverPose;
        Action driveAction = drive.actionBuilder(lastPose)
                .strafeToLinearHeading(thisPose.position, thisPose.heading, baseVelConstraint, baseAccelConstraint)
                .build();
        arm.setTargetPosition(highChamberPrepArm);
        Actions.runBlocking(
                new ParallelAction(
                        driveAction,
                        new SequentialAction(
                                new SleepAction(0.6),
                                (telemetryPacket) -> {
                                    wrist.setPosition(highChamberPrepWrist);
                                    slide.setTargetPosition(highChamberPrepSlide);
                                    return false;
                                }
                        )
                )
        );
        lastPose = thisPose;
        sleep(100);
        slide.setTargetPosition(highChamberDeliverSlide);
        arm.setTargetPosition(highChamberDeliverArm);
        wrist.setPosition(highChamberDeliverWrist);
        while(Math.abs(slide.getCurrentPosition() - highChamberDeliverSlide) < 10){
            sleep(10);
        }
        sleep(500);
        claw.setPosition(clawOpen);
        sleep(100);
        arm.setTargetPosition(10);
        slide.setTargetPosition(10);
        Actions.runBlocking(
                drive.actionBuilder(lastPose)
                        .strafeToConstantHeading(new Vector2d(35.0, highChamberY), baseVelConstraint,baseAccelConstraint)
                        .strafeToConstantHeading(new Vector2d(46.0, -14.0), baseVelConstraint,baseAccelConstraint)
                        .strafeToConstantHeading(new Vector2d(46.0, -50.0), baseVelConstraint,baseAccelConstraint)
                        .strafeToConstantHeading(new Vector2d(50.0, -15.0), baseVelConstraint,baseAccelConstraint)
                        .strafeToConstantHeading(new Vector2d(57.0, -15.0), baseVelConstraint,baseAccelConstraint)
                        .build());
        lastPose = new Pose2d(57.0, -15.0, Math.toRadians(-90.0));
        claw.setPosition(clawOpen);
        wrist.setPosition(pickupWristPosition);
        slide.setTargetPosition(pickupSlidePosition);
        Actions.runBlocking(
                drive.actionBuilder(lastPose)
                        .strafeToConstantHeading(new Vector2d(57.0, -55.0), baseVelConstraint,baseAccelConstraint)
//                .splineToConstantHeading(new Vector2d(35.0, highChamberY), Math.toRadians(90.0), baseVelConstraint,baseAccelConstraint)
//                .splineToConstantHeading(new Vector2d(40.0, -14.00), Math.toRadians(130.0), baseVelConstraint,baseAccelConstraint)
//                .splineToConstantHeading(new Vector2d(45.0, -10.0), Math.toRadians(-180.0), curveVelConstraint,baseAccelConstraint)
//                .splineToConstantHeading(new Vector2d(45.0, -14.0), Math.toRadians(-110.0), curveVelConstraint,baseAccelConstraint)
//                .strafeToConstantHeading(new Vector2d(47.0, -58), baseVelConstraint,baseAccelConstraint)
//                .waitSeconds(0.5)
//                .splineToConstantHeading(new Vector2d(50, -14.0), Math.toRadians(135.0), baseVelConstraint,baseAccelConstraint)
//                .splineToConstantHeading(new Vector2d(52.0, -13.0), Math.toRadians(-180.0), curveVelConstraint,baseAccelConstraint)
//                .splineToConstantHeading(new Vector2d(54.0, -14.0), Math.toRadians(-135.0), curveVelConstraint,baseAccelConstraint)
//                .splineToConstantHeading(new Vector2d(57.0, -58.0), Math.toRadians(-90.0), baseVelConstraint,baseAccelConstraint)
                        .build());
        lastPose = new Pose2d(57.0, -55.0, Math.toRadians(-90.0));


        for (int i = 0; i < 3; i++){
            if ( i != 0) {
                thisPose = new Pose2d(pickupXPosition, pickupYPosition, Math.toRadians(pickupHeading));
                driveAction = drive.actionBuilder(lastPose)
                        .strafeToLinearHeading(thisPose.position, thisPose.heading, baseVelConstraint, baseAccelConstraint)
                        .build();
                arm.setTargetPosition(pickupArmPosition);
                Actions.runBlocking(
                        new ParallelAction(
                                driveAction,
                                new SequentialAction(
                                        (telemetryPacket) -> {
                                            claw.setPosition(clawOpen);
                                            wrist.setPosition(pickupWristPosition);
                                            slide.setTargetPosition(pickupSlidePosition);
                                            return false;
                                        }
                                )
                        )
                );
                lastPose = thisPose;
            }
//            sleep(200);
            claw.setPosition(clawClosed);
            sleep(200);



            thisPose = new Pose2d (highChamberX + 2*(i+1), highChamberY, Math.toRadians(highChamberHeading));
            driveAction = drive.actionBuilder(lastPose)
                    .strafeToLinearHeading(thisPose.position, thisPose.heading, baseVelConstraint, baseAccelConstraint)
                    .build();
            arm.setTargetPosition(highChamberPrepArm);
            Actions.runBlocking(
                    new ParallelAction(
                            driveAction,
                            new SequentialAction(
                                    new SleepAction(1.0),
                                    (telemetryPacket) -> {
                                        arm.setTargetPosition(highChamberPrepArm);
                                        wrist.setPosition(highChamberPrepWrist);
                                        slide.setTargetPosition(highChamberPrepSlide);
                                        return false;
                                    }
                            )
                    )
            );
            lastPose = thisPose;
            slide.setTargetPosition(highChamberDeliverSlide);
            arm.setTargetPosition(highChamberDeliverArm);
            wrist.setPosition(highChamberDeliverWrist);
            while(Math.abs(slide.getCurrentPosition() - highChamberDeliverSlide) < 10){
                sleep(10);
            }
            sleep(500);
            claw.setPosition(clawOpen);
            sleep(100);



        }

//        for (int i = 0; i < 4; i++) {
//            // pickup
//            if ( i > 0){
//                arm.setTargetPosition(armPrepPosition);
//                slide.setTargetPosition(slidePickupPosition);
//                wrist.setPosition(wristPickupPosition);
//                Pose2d farSpikePose = new Pose2d(-7, 23, Math.toRadians(90));
//                Pose2d middleSpikePose = new Pose2d(-18, 23, Math.toRadians(90));
//                Pose2d closeSpikePose = new Pose2d(-21, 26, Math.toRadians(120));
//                if (i == 1){
//                    thisPose = farSpikePose;
//                } else if (i == 2){
//                    thisPose = middleSpikePose;
//                } else if (i == 3){
//                    thisPose = closeSpikePose;
//                }
//
//                if (i == 3){
//                    while(arm.getCurrentPosition() > (armPrepPosition+100)){
//                        sleep(10);
//                    }
//                }
//
//                Actions.runBlocking(
//                        drive.actionBuilder(lastPose)
//                                .strafeToLinearHeading(thisPose.position, thisPose.heading, baseVelConstraint, baseAccelConstraint)
//                                .build());
//                lastPose = thisPose;
//                arm.setTargetPosition(armPickupPosition);
//                while(arm.getCurrentPosition() > (armPickupPosition+100)){
//                    sleep(10);
//                }
//                claw.setPosition(clawClosed);
//                sleep(500);
//            }
//            //deliver
//            Pose2d deliverPose = new Pose2d(-24, 10, Math.toRadians(90));
//            thisPose = deliverPose;
//            arm.setTargetPosition(armHighBucketPosition);
//            wrist.setPosition(wristStraightUp);
//            Actions.runBlocking(
//                    drive.actionBuilder(lastPose)
//                            .strafeToLinearHeading(thisPose.position, thisPose.heading, baseVelConstraint, baseAccelConstraint)
////                        .strafeToConstantHeading(new Vector2d(PLACEONE_X, PLACEONE_Y), baseVelConstraint,baseAccelConstraint)
////                        .splineToConstantHeading(new Vector2d(PLACEONE_X, PLACEONE_Y), Math.toRadians(-90.0), baseVelConstraint,baseAccelConstraint)
//                            //.strafeTo(poseOne.toPose2d().position)
//                            //.strafeTo(poseTwo.toPose2d().position)
//                            //.strafeTo(poseThree.toPose2d().position)
//                            //.strafeTo(poseFour.toPose2d().position)
//                            //.strafeTo(poseFive.toPose2d().position)
//                            .build());
//            lastPose = thisPose;
//            slide.setTargetPosition(slideHighBucketPosition);
//            while(slide.getCurrentPosition() < (slideHighBucketPosition-100)){
//                sleep(10);
//            }
//            wrist.setPosition(0.95);
//            sleep(1000);
//            claw.setPosition(clawOpen);
//            sleep(500);
//
//        }
//        wrist.setPosition(wristStraightUp);
//        sleep(500);
//        arm.setTargetPosition(10);
//        slide.setTargetPosition(10);
//        sleep(2000);




        // --------------------------------------




        if (isStopRequested()) return;
        telemetry.addData("Started Running", " ");
        telemetry.update();
        sleep(startDelay);
        outputLog(drive); //1

    }
    public void outputLog (MecanumDrive drive){
        RobotLog.d("WAY: Current Robot Pose Estimate and time: X: %.03f Y: %.03f Heading: %.03f ms: %.03f iteration: %d", drive.pose.position.x, drive.pose.position.y, Math.toDegrees(drive.pose.heading.real), runtime.milliseconds(), i);
        i++;
    }
}
