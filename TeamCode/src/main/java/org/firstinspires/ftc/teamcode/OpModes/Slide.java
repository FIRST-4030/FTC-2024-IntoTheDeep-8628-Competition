//package org.firstinspires.ftc.teamcode.OpModes;
//
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//
//class Slide {
//    private DcMotorEx slide;
//
//    public Slide(DcMotorEx slideArg) {
//        slide = slideArg;
//    }
//
//    public Action spinUp() {
//        return new Action() {
//            private boolean initialized = false;
//
//            @Override
//            public boolean run(@NonNull TelemetryPacket packet) {
//                if (!initialized) {
//                    motor.setPower(0.8);
//                    initialized = true;
//                }
//
//                double vel = motor.getVelocity();
//                packet.put("shooterVelocity", vel);
//                return vel < 10_000.0;
//            }
//        };
//    }
//}