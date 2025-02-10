package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;

/**
 * This is a wrapper class for Pose2d so that components are configurable before conversion
 */
public class Pose2dWrapper {

    public double x, y, heading;

    /**
     *
     * Pose2dWrapper is a convenience utility used to simplify
     * turning an X, Y, and Heading into a Pose2d object
     *
     * @param x The X coordinate of the pose
     * @param y The Y coordinate of the pose
     * @param heading the Heading (radians) of the pose
     */
    public Pose2dWrapper(double x, double y, double heading){
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    /**
     *
     * @return The stored coordinates as a Pose2d object
     */
    public Pose2d toPose2d(){
        return new Pose2d(x, y, heading);
    }
}
