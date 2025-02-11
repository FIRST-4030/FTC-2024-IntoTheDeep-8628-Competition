package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
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

@Config
@Autonomous
public class MecanumAutoSandbox extends LinearOpMode {

    enum Options { TBD, OneRotation, Sample5 }

    Action thisAction;
    boolean inputComplete = false;
    int increment = 0;
    LogFile detailsLog;
    MecanumDrive drive;
    Options option = Options.TBD;

    public static boolean logDetails = true;
    public static double moveIncrement = 0.0001;
    public static double baseX = 5;
    public static double baseY = 5;
    public static double baseHead = 0;
    public static double step1X = -6;
    public static double step1Y = 55;
    public static double step1Head = 90;
    public static double step2X = -6;
    public static double step2Y = -35;
    public static double step2Head = 30;

    @Override
    public void runOpMode() throws InterruptedException {

        if (logDetails) { detailsLog = new LogFile(LogFile.FileType.Details,"details", "csv"); }

        Pose2dWrapper startPose = new Pose2dWrapper( baseX, baseY, Math.toRadians(baseHead));

        drive = new MecanumDrive(hardwareMap, startPose.toPose2d(), detailsLog, logDetails);
        if (!drive.controlHub.isMacAddressValid()) {
            drive.controlHub.reportBadMacAddress(telemetry,hardwareMap);
            telemetry.update();
        } else {

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

                if (!drive.controlHub.isMacAddressValid()) {
                    drive.controlHub.reportBadMacAddress(telemetry, hardwareMap);
                }
                telemetry.addData("Compiled on:", BuildConfig.COMPILATION_DATE);
                telemetry.addLine();
                telemetry.addData("Option (A-OneRotation, B-Sample5): ", option);
                telemetry.addData("Increment: ", increment);
                telemetry.addData("Press X to finalize values", inputComplete);
                telemetry.update();
            }
        }

        thisAction = drive.actionBuilder(startPose.toPose2d())
                .turn(Math.toRadians(120))
                .build();
        Pose2dWrapper noMovePose = new Pose2dWrapper(baseX+moveIncrement, baseY, Math.toRadians(step1Head));

        waitForStart();
        if (isStopRequested()) return;

        switch (option) {
            case OneRotation:
                Action strafeAction = drive.actionBuilder(startPose.toPose2d())
                        .strafeToLinearHeading(noMovePose.toPose2d().position,noMovePose.toPose2d().heading.toDouble())
                        .build();
                Actions.runBlocking(strafeAction);

                detailsLog.logDelta(noMovePose.toPose2d(),drive.pose);
                break;
            case Sample5:
                detailsLog.log("1,Start");
                Pose2dWrapper step1Pose = new Pose2dWrapper(step1X,step1Y,Math.toRadians(step1Head));
                Pose2dWrapper step2Pose = new Pose2dWrapper(step2X,step2Y,Math.toRadians(step2Head));
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