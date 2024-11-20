package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;

import java.util.Arrays;

@Config
@Disabled
@TeleOp(name = "SplineDemoWithDashboard")
public final class SplineDemoWithDashboard extends LinearOpMode {
    public static double INITIAL_X = -7;
    public static double INITIAL_Y = 62.5;
    public static double START_X = -35;
    public static double START_Y = 5;
    public static double MID_X = -35;
    public static double MID_Y = 35;
    public static double TARGET_X = -55;
    public static double TARGET_Y = 50;
    public static double X1 = -38;
    public static double X2 = -48;
    public static double X3 = -55;

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(INITIAL_X, INITIAL_Y, Math.toRadians(-90.00));
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
            MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);




            waitForStart();
//          57.74, 55.91), Math.toRadians(44.44)
            Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        .splineToConstantHeading(new Vector2d(MID_X, MID_Y), Math.toRadians(-90.0), baseVelConstraint,baseAccelConstraint)
                        .strafeToConstantHeading(new Vector2d(START_X, START_Y), baseVelConstraint,baseAccelConstraint)
                        .strafeToConstantHeading(new Vector2d(X1, START_Y), baseVelConstraint,baseAccelConstraint)
                        .splineToConstantHeading(new Vector2d(TARGET_X, TARGET_Y), Math.toRadians(-130.0), baseVelConstraint,baseAccelConstraint)
                        .splineToConstantHeading(new Vector2d(MID_X, MID_Y), Math.toRadians(-90.0), baseVelConstraint,baseAccelConstraint)
                        .splineToConstantHeading(new Vector2d(START_X, START_Y), Math.toRadians(180.0), curveVelConstraint,baseAccelConstraint)
                        .strafeToConstantHeading(new Vector2d(X2, START_Y), baseVelConstraint,baseAccelConstraint)
                        .splineToConstantHeading(new Vector2d(TARGET_X, TARGET_Y), Math.toRadians(-130.0), baseVelConstraint,baseAccelConstraint)
                        .splineToConstantHeading(new Vector2d(MID_X, MID_Y), Math.toRadians(-90.0), baseVelConstraint,baseAccelConstraint)
                        .splineToConstantHeading(new Vector2d(START_X, START_Y), Math.toRadians(180.0), curveVelConstraint,baseAccelConstraint)
                        .strafeToConstantHeading(new Vector2d(X3, START_Y), baseVelConstraint,baseAccelConstraint)
                        .splineToConstantHeading(new Vector2d(TARGET_X, TARGET_Y), Math.toRadians(-130.0), baseVelConstraint,baseAccelConstraint)
                        .splineToConstantHeading(new Vector2d(MID_X, MID_Y), Math.toRadians(-90.0), baseVelConstraint,baseAccelConstraint)
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
