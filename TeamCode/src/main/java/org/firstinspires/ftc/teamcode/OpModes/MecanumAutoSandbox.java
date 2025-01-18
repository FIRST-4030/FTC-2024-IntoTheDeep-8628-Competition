package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.BuildConfig;
import org.firstinspires.ftc.teamcode.LogFile;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Pose2dWrapper;
import org.firstinspires.ftc.teamcode.gamepad.InputAutoMapper;
import org.firstinspires.ftc.teamcode.gamepad.InputHandler;

import java.util.Arrays;

@Config
@Autonomous
public class MecanumAutoSandbox extends LinearOpMode {

    enum Options { TBD, OneRotation, Sample5 }

    Action thisAction;
    boolean inputComplete = false;
    int increment = 0;
    LogFile detailsLog;
    MecanumDrive drive;
    boolean inputComplete = false;
    Options option = Options.TBD;
    int increment = 0;
    Action thisAction;

    public static boolean logDetails = true;
    public static double baseVel = 40.0;
    public static double accelMin = -20.0;
    public static double accelMax = 50.0;

    @Override
    public void runOpMode() throws InterruptedException {


        VelConstraint baseVelConstraint = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(baseVel),
                new AngularVelConstraint(Math.PI / 2)
        ));
        AccelConstraint baseAccelConstraint = new ProfileAccelConstraint(accelMin, accelMax);

        TurnConstraints turnConstraints = new TurnConstraints(Math.toRadians(90),Math.toRadians(90),Math.toRadians(90));

        Pose2dWrapper startPose = new Pose2dWrapper( baseX, baseY, Math.toRadians(baseHead));

        drive = new MecanumDrive(hardwareMap, startPose.toPose2d(), detailsLog, logDetails);

        InputHandler inputHandler;
        ElapsedTime runtime = new ElapsedTime();
        ElapsedTime inputTimer = new ElapsedTime();

        runtime.reset();

        inputHandler = InputAutoMapper.normal.autoMap(this);

        while (!inputComplete) {
            inputHandler.loop();
            if (inputTimer.milliseconds() > 250) {

                if (inputHandler.up("D1:X")) {
                    inputComplete = true;
                    inputTimer.reset();
                }

                if (inputHandler.up("D1:A")) {
                    option = Options.OneRotation;
                    inputTimer.reset();
                }

                if (inputHandler.up("D1:B")) {
                    option = Options.Sample5;
                    inputTimer.reset();
                }

                if (inputHandler.up("D1:LT")) {
                    increment++;
                    inputTimer.reset();
                }

                if (inputHandler.up("D1:RT")) {
                    increment--;
                    inputTimer.reset();
                }
            }

            telemetry.addData("Compiled on:", BuildConfig.COMPILATION_DATE);
            telemetry.addLine();
            telemetry.addData("Increment: ", increment);
            telemetry.addData("Press X to finalize values", inputComplete);
            telemetry.update();
        }

        thisAction = drive.actionBuilder(startPose.toPose2d())
                .turn(Math.toRadians(120))
                .build();
        Pose2dWrapper noMovePose = new Pose2dWrapper(baseX+moveIncrement, baseY, Math.toRadians(step1Head));

        waitForStart();
        if (isStopRequested()) return;

        switch (option) {
            case OneRotation:
                        .build();
                break;
            case Sample5:
                detailsLog.log("1,Start");
                Pose2dWrapper step1Pose = new Pose2dWrapper(step1X, step1Y, Math.toRadians(step1Head));
                Pose2dWrapper step2Pose = new Pose2dWrapper( step2X, step2Y, Math.toRadians(step2Head) );
                thisAction = drive.actionBuilder(startPose.toPose2d())
                        .splineToLinearHeading(step1Pose.toPose2d(),step1Pose.heading)
                        .waitSeconds(2.0)
                        .strafeToConstantHeading(step2Pose.toPose2d().position)
                        .build();
                Actions.runBlocking(thisAction);
                break;
        }
    }
}