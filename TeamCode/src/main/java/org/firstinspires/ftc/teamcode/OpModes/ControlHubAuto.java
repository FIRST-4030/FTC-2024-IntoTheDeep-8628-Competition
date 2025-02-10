package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.BuildConfig;
import org.firstinspires.ftc.teamcode.ControlHub;

@Autonomous(name="ControlHubAuto")
public class ControlHubAuto extends LinearOpMode {

    ControlHub controlHub;

    @Override
    public void runOpMode() throws InterruptedException {

        controlHub = new ControlHub();

        telemetry.addData("Compiled on:", BuildConfig.COMPILATION_DATE);
        if (!controlHub.isMacAddressValid()) {
            controlHub.reportBadMacAddress(telemetry,hardwareMap);
        } else {

            telemetry.addData("MAC Address:", controlHub.getMacAddress());
            telemetry.addData("Network Name:", controlHub.getNetworkName());
            telemetry.addData("Comment:", controlHub.getComment());
        }
        telemetry.update();

        waitForStart();

//        while (opModeIsActive()) {}
    }
}
