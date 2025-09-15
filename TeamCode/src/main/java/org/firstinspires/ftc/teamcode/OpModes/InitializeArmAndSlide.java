package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.BuildConfig;

public class InitializeArmAndSlide {

    public static boolean initializeArmAndSlide(Telemetry telemetry, Servo wristRotation, Servo claw, Servo wrist, DcMotor slide, DcMotor arm, TouchSensor slideTouchSensor, TouchSensor armTouchSensor) {
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slide.setPower(-0.7);
        arm.setPower(0.0);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        wrist.setPosition(0.8567);
        wrist.setPosition(0.4);
        wristRotation.setPosition(0.5);
        claw.setPosition(0.84);
        while (true){
            if (slideTouchSensor.isPressed()){
                slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                slide.setPower(1.0);
//        if(reverse) {arm.setDirection(DcMotorSimple.Direction.REVERSE);}
                slide.setTargetPosition(10);
                slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        tickLimit = maxVal;
                break;
            }
            telemetry.addLine("slide moving to zero");
            telemetry.addData("slide sensor pushed", slideTouchSensor.isPressed());
            telemetry.addData("arm sensor pushed", armTouchSensor.isPressed());
            telemetry.update();
        }
        arm.setPower(-0.5);
        while (true) {
            if (armTouchSensor.isPressed()) {
                arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                arm.setPower(1.0);
//          if(reverse) {arm.setDirection(DcMotorSimple.Direction.REVERSE);}
                arm.setTargetPosition(10);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                telemetry.addLine("initialization successful");
                telemetry.update();
//        tickLimit = maxVal;
                return true;
            }
            telemetry.addLine("arm moving to zero");
            telemetry.addData("slide sensor pushed", slideTouchSensor.isPressed());
            telemetry.addData("arm sensor pushed", armTouchSensor.isPressed());
            String compilationDate = BuildConfig.COMPILATION_DATE;
            telemetry.addData("Compiled on:", compilationDate);
            telemetry.update();
        }

    }
}
