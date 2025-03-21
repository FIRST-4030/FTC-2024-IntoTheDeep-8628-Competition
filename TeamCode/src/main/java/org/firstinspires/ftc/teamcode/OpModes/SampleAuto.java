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

import org.firstinspires.ftc.teamcode.BuildConfig;
import org.firstinspires.ftc.teamcode.LogFile;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Pose2dWrapper;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.teamcode.ComputerVision;

import java.util.Arrays;

@Config
@Autonomous(name = "SampleAuto")
public final class SampleAuto extends LinearOpMode {
    // 1.455 slide
    // 1.374
    // driver assist high bucket arm, slide and wrist movement
    public static int slideHighBucketPosition = 3263;
    public static int armHighBucketPosition = 4382;
    public static int armPrepPosition = 800;
    public static int armPickupPosition = 450;
    public static int slidePickupPosition = 745;
    public static double wristPickupPosition = 0.15;
    public static double wristStraightUp = 0.35;
    public static double clawOpen = 0.04;
    public static double clawClosed = 0.84;
    public static int sleepAfterClawClosed = 500;
    public static int sleepAfterClawOpen = 500;
    public static int sleepAfterWristDeliver = 1500;
    public static double wristDeliverPos = 0.8567;
    public static double maxTransVelocity = 30.0;
    public static double accelMin = -20.0;
    public static double accelMax = 40.0;

    public static double poseFarSpikeX = -8.5;
    public static double poseFarSpikeY = 22;
    public static double poseMiddleSpikeX = -19.5;
    public static double poseMiddleSpikeY = 22;
    public static double poseCloseSpikeX = -20;
    public static double poseCloseSpikeY = 24;
    public static double poseDeliverX = -23; // old -24
    public static double poseDeliverY = 8; // old 12
    public static double poseDeliverHeading = 45; // old 90
    public static double parkPoseX = 6;
    public static double parkPoseY = 55;
    public static int parkArm = 1900;
    public static int parkSlide = 1690;
    public static double maxRotationSpeed = 67;
    public static boolean logAction = false;
    public static boolean logDetails = false;

    LogFile sampleAction;
    LogFile detailsLog;
    int detailsFilter = 1;
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

        if (logAction) { sampleAction = new LogFile(LogFile.FileType.Action,"sample", "csv" ); }
        if (logDetails) { detailsLog = new LogFile(LogFile.FileType.Details,"details", "csv" ); }

        DcMotor slide = hardwareMap.dcMotor.get("slide");
        DcMotor arm = hardwareMap.dcMotor.get("arm");
        Servo claw = hardwareMap.servo.get("claw");
        Servo wrist = hardwareMap.servo.get("wrist");
        Servo wristRotation = hardwareMap.servo.get("wristRotation");
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

        telemetry.addData("Compiled on:", BuildConfig.COMPILATION_DATE);
        telemetry.update();

        VelConstraint baseVelConstraint = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(maxTransVelocity),
                new AngularVelConstraint(Math.toRadians(maxRotationSpeed))
        ));
        VelConstraint curveVelConstraint = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(20.0),
                new AngularVelConstraint(Math.toRadians(maxRotationSpeed))
        ));
        AccelConstraint baseAccelConstraint = new ProfileAccelConstraint(accelMin, accelMax);
        AccelConstraint slowAccelConstraint = new ProfileAccelConstraint(-20, 30);

        Pose2dWrapper startPose = new Pose2dWrapper(0, -0.5, Math.toRadians(90));

        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose.toPose2d(), detailsLog, logDetails);
        if (!drive.controlHub.isMacAddressValid()) {
            drive.controlHub.reportBadMacAddress(telemetry,hardwareMap);
        }

        InitializeArmAndSlide.initializeArmAndSlide(telemetry, wristRotation, claw, wrist, slide, arm, slideTouchSensor, armTouchSensor);

        waitForStart();
//                        .splineToConstantHeading(new Vector2d(-33.0, 30.00), Math.toRadians(-90.0), baseVelConstraint,baseAccelConstraint)
        Pose2d lastPose = startPose.toPose2d();
        Pose2d thisPose = lastPose;
        for (int i = 0; i < 4; i++) {
            // pickup
            if ( i > 0){
                arm.setTargetPosition(armPrepPosition);
                wrist.setPosition(wristPickupPosition);
                if ( i == 3){
                    sleep(100);
                    slide.setTargetPosition(slidePickupPosition);
                }
                Pose2d farSpikePose = new Pose2d(poseFarSpikeX, poseFarSpikeY, Math.toRadians(90));
                Pose2d middleSpikePose = new Pose2d(poseMiddleSpikeX, poseMiddleSpikeY, Math.toRadians(90));
                Pose2d closeSpikePose = new Pose2d(poseCloseSpikeX, poseCloseSpikeY, Math.toRadians(120));

                if (i == 1){
                    thisPose = farSpikePose;
                    if (logAction) { sampleAction.logAction( thisPose, "Far Spike" ); }
                    if (logDetails) { detailsLog.log( detailsFilter + "," + "Start Far Spike" ); }
                } else if (i == 2){
                    thisPose = middleSpikePose;
                    if (logAction) { sampleAction.logAction( thisPose, "Middle Spike" ); }
                    if (logDetails) { detailsLog.log( detailsFilter + "," + "Start Middle Spike" ); }
                } else if (i == 3){
                    thisPose = closeSpikePose;
                    if (logAction) { sampleAction.logAction( thisPose, "Close Spike" ); }
                    if (logDetails) { detailsLog.log( detailsFilter + "," + "Start Close Spike" ); }
                    MecanumDrive.errorTolerance = 100;
                }

                if (i == 3){
                    while(arm.getCurrentPosition() > (armPrepPosition+100)){
                        sleep(10);
                    }
                }
                if (logDetails) { detailsLog.log( detailsFilter + "," + "Start Drive to End" ); }
                Action driveAction = drive.actionBuilder(lastPose)
                        .strafeToLinearHeading(thisPose.position, thisPose.heading, baseVelConstraint, baseAccelConstraint)
                        .build();
                Actions.runBlocking(
                        new ParallelAction(
                                driveAction,
                                new SequentialAction(
                                        new SleepAction(0.1),
                                        (telemetryPacket) -> {
                                            slide.setTargetPosition(slidePickupPosition);
                                            return false;
                                        }
                                )
                        )
                );
                if (logDetails) { detailsLog.log( detailsFilter + "," + "End Drive to End" ); }
                lastPose = thisPose;
                if (logAction) {
                    sampleAction.log( " " );  // Add a blank line
                    sampleAction.logAction( drive.pose );
                }

                MecanumDrive.errorTolerance = 1.5;
                arm.setTargetPosition(armPickupPosition);
                while(arm.getCurrentPosition() > (armPickupPosition+100)){
                    sleep(10);
                }
                sleep(100);
                claw.setPosition(clawClosed);
                sleep(sleepAfterClawClosed);
            }
            //deliver
            Pose2d deliverPose;// = new Pose2d(poseDeliverX+(i==0?-6:0), poseDeliverY, poseDeliverHeading);
            deliverPose = new Pose2d(poseDeliverX, poseDeliverY, poseDeliverHeading);

            thisPose = deliverPose;
            if (logAction) { sampleAction.logAction( thisPose, "Deliver" ); }
            if (logDetails) { detailsLog.log( detailsFilter + "," + "Start Deliver Sample" ); }
            arm.setTargetPosition(armHighBucketPosition);
            wrist.setPosition(wristStraightUp);
            slide.setTargetPosition(slideHighBucketPosition);
            Actions.runBlocking(
                    drive.actionBuilder(lastPose)
                            .strafeToLinearHeading(thisPose.position, thisPose.heading, baseVelConstraint, baseAccelConstraint)
//                        .strafeToConstantHeading(new Vector2d(PLACEONE_X, PLACEONE_Y), baseVelConstraint,baseAccelConstraint)
//                        .splineToConstantHeading(new Vector2d(PLACEONE_X, PLACEONE_Y), Math.toRadians(-90.0), baseVelConstraint,baseAccelConstraint)
                            //.strafeTo(poseOne.toPose2d().position)
                            //.strafeTo(poseTwo.toPose2d().position)
                            //.strafeTo(poseThree.toPose2d().position)
                            //.strafeTo(poseFour.toPose2d().position)
                            //.strafeTo(poseFive.toPose2d().position)
                            .build());
            if (logDetails) { detailsLog.log( detailsFilter + "," + "End Deliver Sample" ); }
            lastPose = thisPose;
            if (logAction) { sampleAction.logAction( drive.pose ); }
            if (logDetails) { detailsLog.log( detailsFilter + "," + "Start Slide to Bucket Position" ); }
            while(slide.getCurrentPosition() < (slideHighBucketPosition-100)){
                sleep(10);
            }
            if (logDetails) { detailsLog.log( detailsFilter + "," + "End Slide to Bucket Position" ); }
            wrist.setPosition(wristDeliverPos);
            sleep(sleepAfterWristDeliver);
            claw.setPosition(clawOpen);
            sleep(sleepAfterClawOpen);
        }
        wrist.setPosition(wristStraightUp);
        arm.setTargetPosition(parkArm);
        sleep(100);
        slide.setTargetPosition(parkSlide);
        sleep(100);
        wrist.setPosition(0.95);
        Pose2d parkPose = new Pose2d (parkPoseX,parkPoseY, Math.toRadians(0));
        if (logAction) {
            sampleAction.log( " " );  // Add a blank line
            sampleAction.logAction( parkPose, "Park" );
        }
        if (logDetails) { detailsLog.log( detailsFilter + "," + "Start Park" ); }
        Actions.runBlocking(
                drive.actionBuilder(lastPose)
                        .splineTo(parkPose.position, parkPose.heading, curveVelConstraint, baseAccelConstraint)
                        .build());
        lastPose = parkPose;
        if (logAction) { sampleAction.logAction( drive.pose ); }
        if (logDetails) { detailsLog.log( detailsFilter + "," + "End Park" ); }

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
