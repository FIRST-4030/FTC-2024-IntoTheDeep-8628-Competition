package org.firstinspires.ftc.teamcode.OpModes;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static org.firstinspires.ftc.teamcode.OpModes.FieldCentricMecanumTeleOp.clawClose;
import static org.firstinspires.ftc.teamcode.OpModes.FieldCentricMecanumTeleOp.clawOpen;
import static java.lang.Math.atan2;
import static java.lang.Thread.sleep;

import com.acmerobotics.roadrunner.ftc.Encoder;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.OpModes.LinearRegression2;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;

public class PickupSample {
    static class ReturnResult {
        int armTargetPosition;
        int slideTargetPosition;
        double wristTargetPosition;
        double wristRotationTargetPosition;
        static boolean pickedUp = false;
    }
    public static ReturnResult pickupSample(IMU imu, Optional<Double> initialAngle, int scanArm, int scanSlide, double scanWrist,
                                            DcMotor arm, DcMotor slide, Servo wrist, Servo claw, Limelight3A limelight,
                                            Telemetry telemetry, Encoder par0, Encoder par1, Encoder perp, DcMotor frontLeftMotor,
                                            DcMotor frontRightMotor, DcMotor backLeftMotor, DcMotor backRightMotor,
                                            int pickupArm, int pickupSlide, double pickupWrist,
                                            Servo wristRotation) {
        if (initialAngle == null || !initialAngle.isPresent()) {
            initialAngle = Optional.of(getImuAngle(imu).getYaw(AngleUnit.DEGREES));
        }
        double currentAngle = getImuAngle(imu).getYaw(AngleUnit.DEGREES);
        double error = initialAngle.get() - currentAngle;
        if (error > 180) {
            error -= 180;
        } else if (error < -180) {
            error += 180;
        }
        double limelightTurnPower = error / 45;

        int armTargetPosition = scanArm;
        arm.setTargetPosition(armTargetPosition);
        int slideTargetPosition = scanSlide;
        slide.setTargetPosition(slideTargetPosition);
        double wristTargetPosition = scanWrist;
        wrist.setPosition(wristTargetPosition);
        claw.setPosition(clawOpen);
        LLResult result = limelight.getLatestResult();
        double tx;
        double ty;
        if (result != null && result.isValid()) {
            List<LLResultTypes.ColorResult> colorResults = result.getColorResults();
            List<List<Double>> corners = null;
            for (LLResultTypes.ColorResult cr : colorResults) {
                corners = cr.getTargetCorners();
            }
            telemetry.addData("corners", corners);
            tx = result.getTx();
            ty = result.getTy();
            telemetry.addData("tx", tx);
            telemetry.addData("ty", ty);
            List<Double> xCorners = new ArrayList<>();
            List<Double> yCorners = new ArrayList<>();
            if (corners != null) {
                for (int i = 0; i < corners.size(); i++) {
                    xCorners.add(i, corners.get(i).get(0));
                }
                telemetry.addData("xCorners", xCorners);
                for (int i = 0; i < corners.size(); i++) {
                    yCorners.add(i, corners.get(i).get(1));
                }
                telemetry.addData("yCorners", yCorners);
            }
            double[] xValues = xCorners.stream().mapToDouble(Double::doubleValue).toArray();
            double[] yValues = yCorners.stream().mapToDouble(Double::doubleValue).toArray();
            if (xValues.length != yValues.length) {
                throw new IllegalArgumentException("array lengths not equal");
            }

            for (int j = xValues.length; j > 2; j--) {
                double leastSquaredDistance = 9999999999.0; //set to a large value that a real number would not be larger
                int closestPoint1 = -1;
                int closestPoint2 = -1;
                double[] newPoint = null;
                for (int i = 0; i < xValues.length; i++) {
                    for (int k = 0; k < xValues.length; k++) {
                        if (i != k) {
                            double xDifference = Math.abs(xValues[k]) - Math.abs(xValues[i]);
                            double yDifference = Math.abs(yValues[k]) - Math.abs(yValues[i]);
                            double distance = (xDifference * xDifference + yDifference * yDifference);
                            if (distance < leastSquaredDistance) {
                                leastSquaredDistance = distance;
                                closestPoint1 = i;
                                closestPoint2 = k;
                                double xAverage = (xValues[i] + xValues[k]) / 2;
                                double yAverage = (yValues[i] + yValues[k]) / 2;
                                newPoint = new double[]{xAverage, yAverage};
                            }
                        }
                    }
                }
                ArrayList<Double> newXvalues = new ArrayList<>();
                if (newPoint != null) {
                    newXvalues.add(0, newPoint[0]);
                    for (int i = 0; i < xValues.length; i++) {
                        if (i != closestPoint1 && i != closestPoint2) {
                            newXvalues.add(newXvalues.size(), xValues[i]);
                        }
                    }
                    xValues = newXvalues.stream().mapToDouble(Double::doubleValue).toArray();
                }
                ArrayList<Double> newYvalues = new ArrayList<>();
                if (newPoint != null) {
                    newYvalues.add(0, newPoint[1]);
                    for (int i = 0; i < yValues.length; i++) {
                        if (i != closestPoint1 && i != closestPoint2) {
                            newYvalues.add(newYvalues.size(), yValues[i]);
                        }
                    }
                    yValues = newYvalues.stream().mapToDouble(Double::doubleValue).toArray();
                }
            }
            double slope = (yValues[0] - yValues[1]) / (xValues[0] - xValues[1]);
            double theta = atan2((yValues[0] - yValues[1]), (xValues[0] - xValues[1]));
                    /* theta;wrist
                    pi/2; 0.5
                    0.8; 0.38
                    0; 0.22
                    2.42; 0.66
                    -0.12; 0.76
                    0.06; 0.25
                     */

            telemetry.addData("xValues", xValues);
            telemetry.addData("yValues", yValues);
            telemetry.addData("xValues", Arrays.toString(xValues));
            telemetry.addData("yValues", Arrays.toString(yValues));
            telemetry.addData("theta", theta);
            telemetry.addData("slope", slope);
            telemetry.addData("regressionSlope", new LinearRegression2(xValues, yValues).getSlope());
            telemetry.addData("intercept", new LinearRegression2(xValues, yValues).getIntercept());
//                    double slope = LinearRegression.slope();
//                    double intercept = LinearRegression.intercept();
            double vel0 = par0.getPositionAndVelocity().velocity;
            double vel1 = par1.getPositionAndVelocity().velocity;
            double vel2 = perp.getPositionAndVelocity().velocity;
            double parallelVelocity = (vel0 + vel1) / 2;
            double velocity = Math.sqrt(parallelVelocity * parallelVelocity + vel2 * vel2);
            telemetry.addData("velocity", velocity);
            if ((Math.abs(tx) > 3 || Math.abs(ty) > 3) || velocity > 500) {
                move(tx, -ty, frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor, limelightTurnPower, telemetry);
            } else {
                armTargetPosition = pickupArm;
                arm.setTargetPosition(armTargetPosition);
                slideTargetPosition = pickupSlide;
                slide.setTargetPosition(slideTargetPosition);
                wristTargetPosition = pickupWrist;
                wrist.setPosition(wristTargetPosition);
                double wristRotationPosition = (0.44 / 2.42) * theta + 0.22;
                while (wristRotationPosition > 0.75) {
                    wristRotationPosition -= 0.5;
                }
                while (wristRotationPosition < 0.25) {
                    wristRotationPosition += 0.5;
                }
                double wristRotationTargetPosition = wristRotationPosition;
                wristRotation.setPosition(wristRotationTargetPosition);
                backLeftMotor.setPower(0);
                backRightMotor.setPower(0);
                frontLeftMotor.setPower(0);
                frontRightMotor.setPower(0);

                ReturnResult returnResult = new ReturnResult();
                returnResult.slideTargetPosition = slideTargetPosition;
                returnResult.wristRotationTargetPosition = wristRotationTargetPosition;
                returnResult.armTargetPosition = armTargetPosition;
                returnResult.wristTargetPosition = wristTargetPosition;
                returnResult.pickedUp = true;
                return returnResult;


            }
        }

        return null;
    }


    public static YawPitchRollAngles getImuAngle(IMU imu) {
        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
        return angles;
    }

    public AngularVelocity getAngularVelocity(IMU imu) {
        AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
        return angularVelocity;
    }

    public static void move(double x /* degrees to the left */, double y, DcMotor frontLeftMotor, DcMotor backLeftMotor,
                            DcMotor frontRightMotor, DcMotor backRightMotor, double turnPower, Telemetry telemetry) {
        double minPower = 0.3;
        double powerLeft = 0;
        if (x > 3) {
            powerLeft = Math.max(minPower, x / 20);
        } else if (x < -3) {
            powerLeft = Math.min(-minPower, x / 20);
        }
        double powerForward = 0;
        if (y > 3) {
            powerForward = Math.max(minPower, y / 20);
        } else if (y < -3) {
            powerForward = Math.min(-minPower, y / 20);
        }
        telemetry.addData("powerLeft", powerLeft);
        telemetry.addData("powerForward", powerForward);

        double frontLeftPower = powerForward - powerLeft + turnPower;
        double backLeftPower = powerForward + powerLeft + turnPower;
        double frontRightPower = powerForward + powerLeft - turnPower;
        double backRightPower = powerForward - powerLeft - turnPower;

        telemetry.addData("frontLeftPower", frontLeftPower);
        telemetry.addData("backLeftPower", backLeftPower);
        telemetry.addData("frontRightPower", frontRightPower);
        telemetry.addData("backRightPower", backRightPower);

        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);

    }
}