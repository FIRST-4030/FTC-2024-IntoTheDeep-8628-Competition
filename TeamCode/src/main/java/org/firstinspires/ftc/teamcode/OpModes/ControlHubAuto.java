package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.BuildConfig;
import org.firstinspires.ftc.teamcode.ControlHub;

@Disabled
@Autonomous(name="ControlHubAuto")
public class ControlHubAuto extends LinearOpMode {

    ControlHub controlHub;

    @Override
    public void runOpMode() throws InterruptedException {

        if (opModeInInit()) {

            controlHub = new ControlHub();

            String compilationDate = BuildConfig.COMPILATION_DATE;

            telemetry.addData("Compiled on:", compilationDate);
            telemetry.addData("MAC Address:", controlHub.getMacAddress());
            telemetry.addData("Network Name:", controlHub.getNetworkName());
            telemetry.addData("Comment:", controlHub.getComment());
            telemetry.update();
        }

        waitForStart();

//        while (opModeIsActive()) {}
    }
}
