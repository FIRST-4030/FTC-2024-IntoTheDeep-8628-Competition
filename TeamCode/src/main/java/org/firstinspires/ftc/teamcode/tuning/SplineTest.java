package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.LogFile;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.TankDrive;

import java.util.Arrays;

public final class SplineTest extends LinearOpMode {
    public static boolean logDetails = false;

    LogFile detailsLog;

    @Override
    public void runOpMode() throws InterruptedException {


        Pose2d beginPose = new Pose2d(-31.00, 62.5, Math.toRadians(-90.00));
//        VelConstraint baseVelConstraint = new MinVelConstraint(Arrays.asList(
//                new TranslationalVelConstraint(100.0),
//                new AngularVelConstraint(Math.PI / 2)
//        ));
//        AccelConstraint baseAccelConstraint = new ProfileAccelConstraint(-30.0, 100.0);
        //hardwareMap.get(WebcamName.class, "Webcam 1")
        VelConstraint baseVelConstraint = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(40.0),
                new AngularVelConstraint(Math.PI / 2)
        ));
        VelConstraint curveVelConstraint = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(15.0),
                new AngularVelConstraint(Math.PI / 2)
        ));
        AccelConstraint baseAccelConstraint = new ProfileAccelConstraint(-20.0, 50.0);

        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose, detailsLog, logDetails );

            waitForStart();
//          57.74, 55.91), Math.toRadians(44.44)
            Actions.runBlocking(
                drive.actionBuilder(beginPose)
//                        .lineToY(-30,baseVelConstraint,baseAccelConstraint)
//                        .build()
//                        .splineToConstantHeading(new Vector2d(-33.0, 30.00), Math.toRadians(-90.0), baseVelConstraint,baseAccelConstraint)
//                        .splineToConstantHeading(new Vector2d(-33.0, 11.0), Math.toRadians(-90.0), baseVelConstraint,baseAccelConstraint)
//                        .strafeToConstantHeading(new Vector2d(-47.0, 11.0), baseVelConstraint,baseAccelConstraint)
//                        .strafeToConstantHeading(new Vector2d(-47.0, 57.91), baseVelConstraint,baseAccelConstraint)
//                        .waitSeconds(0.5)
//                        .splineToConstantHeading(new Vector2d(-47, 11.0), Math.toRadians(-90.0), baseVelConstraint,baseAccelConstraint)
//                        .strafeToConstantHeading(new Vector2d(-57.0, 11.0), baseVelConstraint,baseAccelConstraint)
//                        .strafeToConstantHeading(new Vector2d(-57.0, 57.91), baseVelConstraint,baseAccelConstraint)
//                        .waitSeconds(0.5)
//                        .strafeToConstantHeading(new Vector2d(-57, 11.0), baseVelConstraint,baseAccelConstraint)
//                        .strafeToConstantHeading(new Vector2d(-64.0, 11.0), baseVelConstraint,baseAccelConstraint)
//                        .strafeToConstantHeading(new Vector2d(-64.0, 57.91), baseVelConstraint,baseAccelConstraint)
//                        .build()
                        .splineToConstantHeading(new Vector2d(-31.0, 30.00), Math.toRadians(-90.0), baseVelConstraint,baseAccelConstraint)
                        .splineToConstantHeading(new Vector2d(-35.0, 14.00), Math.toRadians(-130.0), baseVelConstraint,baseAccelConstraint)
                        .splineToConstantHeading(new Vector2d(-40.0, 10.0), Math.toRadians(180.0), curveVelConstraint,baseAccelConstraint)
                        .splineToConstantHeading(new Vector2d(-45.0, 14.0), Math.toRadians(110.0), curveVelConstraint,baseAccelConstraint)
                        .splineToConstantHeading(new Vector2d(-47.0, 57.91), Math.toRadians(90), baseVelConstraint,baseAccelConstraint)
                        .waitSeconds(0.5)
                        .splineToConstantHeading(new Vector2d(-50, 14.0), Math.toRadians(-135.0), baseVelConstraint,baseAccelConstraint)
                        .splineToConstantHeading(new Vector2d(-52.0, 13.0), Math.toRadians(180.0), curveVelConstraint,baseAccelConstraint)
                        .splineToConstantHeading(new Vector2d(-54.0, 14.0), Math.toRadians(135.0), curveVelConstraint,baseAccelConstraint)
                        .splineToConstantHeading(new Vector2d(-57.0, 57.91), Math.toRadians(90.0), baseVelConstraint,baseAccelConstraint)
                        .waitSeconds(0.5)
                        .splineToConstantHeading(new Vector2d(-57, 12.0), Math.toRadians(-90.0), baseVelConstraint,baseAccelConstraint)
                        .splineToConstantHeading(new Vector2d(-63.0, 12.0), Math.toRadians(-180.0), baseVelConstraint,baseAccelConstraint)
                        .strafeToConstantHeading(new Vector2d(-63.0, 57.91), baseVelConstraint,baseAccelConstraint)
                        .build()
                );
        } else if (TuningOpModes.DRIVE_CLASS.equals(TankDrive.class)) {
            TankDrive drive = new TankDrive(hardwareMap, beginPose);

            waitForStart();

            Actions.runBlocking(
                    drive.actionBuilder(beginPose)
                            .splineTo(new Vector2d(30, 30), Math.PI / 2)
                            .splineTo(new Vector2d(0, 60), Math.PI)
                            .build());
        } else {
            throw new RuntimeException();
        }
    }
}
