package org.firstinspires.ftc.teamcode;

import android.os.Environment;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;

public class LogFile {
    private FileWriter logWriter;
    private String label = null;

    private double startHeading = 0.0;
    private double startPoseX   = 0.0;
    private double startPoseY   = 0.0;

    private final long absoluteStartTime;
    private long startTime;
    private long deltaTime;

    public String fullPath = null;

    /**
     * Note: For this method to work you have to be sure that the
     *       folder "/sdcard/FIRST/logs" exists on the Control Hub
     *
     * @param prefix - root name of the file that will have a time stamp added
     * @param fileType - file extension (typically "csv" or "txt")
     */
    public LogFile( String prefix, String fileType ) {

        absoluteStartTime = System.currentTimeMillis();

        String logFolder = Environment.getExternalStorageDirectory().getPath(); // /storage/emulated/0 also maps to /sdcard

        // Define a unique file name
        SimpleDateFormat dateFormat = new SimpleDateFormat("yyyy-MM-dd HH:mm", Locale.US );
        String timestamp = dateFormat.format(new Date());

        String fileName = logFolder + "/FIRST/logs/" + prefix + "_" + timestamp + "." + fileType;

        try {
            File logFile = new File(fileName);
            fullPath = logFile.getPath();
            if (!logFile.exists()) {
                logWriter = new FileWriter(logFile);
            } else {
                logWriter = new FileWriter(logFile, true); // Append mode
            }
        } catch (IOException e) {
            e.printStackTrace();
        }

        /*
         ** Create a one line file that holds the name of the current log file
         ** so that subsequent applications can get a path to the most recent file
         */
        String oneLinerFile = logFolder + "/FIRST/logs/" + prefix + ".txt";
        try {
            File oneLiner = new File(oneLinerFile);
            FileWriter oneLineWriter = new FileWriter(oneLiner);
            oneLineWriter.write(fileName + "\n");
            oneLineWriter.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    /**
     *
     * @param message - string of characters to be written with a appended <NL>
     */
    public void log(String message) {
        try {
            logWriter.write(message + "\n");
            logWriter.flush();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public void logDetailsTitles() {
        String message = "Filter,Time Stamp (ms),Ang Vel,Linear Vel x,Linear Vel y,Err x,Err y,Heading (deg)";
        log( message );
    }

    /**
     *
     * @param vector - heading and position
     * @param error - error in x, y, & heading
     */
    public void logDetails(PoseVelocity2d vector, Pose2d error) {
        deltaTime = System.currentTimeMillis() - absoluteStartTime;
        int filter = 0;
        String message = filter + "," +
                String.format(Locale.US, "%.4f", (deltaTime/1000.0)) + "," +
                String.format(Locale.US, "%.4f", vector.angVel) + "," +
                String.format(Locale.US, "%.4f", vector.linearVel.x) + "," +
                String.format(Locale.US, "%.4f", vector.linearVel.y) + "," +
                String.format(Locale.US, "%.4f", error.position.x) + "," +
                String.format(Locale.US, "%.4f", error.position.y) + "," +
                String.format(Locale.US, "%.4f", error.heading.real);

        log( message );
    }

    /*
     * All methods below this line are specific to the current year's competition
     *
     * Feel free to remove/alter any of them for a new season
     */
    public void logSampleTitles() {
        String message = ",Delta,Start,Start,Start,End,End,End,Delta,Delta,Delta";
        log( message );
        message = ",Time,Heading,X,Y,Heading,X,Y,Heading,X,Y";
        log( message );
    }

    /**
     *
     * @param isStart - boolean value used to capture the start of a sample
     * @param label - text string used to describe the sample
     * @param pose - x & y position of the robot
     */
    public void logSample( boolean isStart, String label, Pose2d pose ) {

        if (isStart) {
            this.label   = label;
            startTime    = System.currentTimeMillis();
            startHeading = Math.toDegrees(pose.heading.toDouble());
            startPoseX   = pose.position.x;
            startPoseY   = pose.position.y;
        } else {
            deltaTime  = System.currentTimeMillis() - startTime;
            double endHeading = Math.toDegrees(pose.heading.toDouble());
            double endPoseX   = pose.position.x;
            double endPoseY   = pose.position.y;

            String message = this.label + "," +
                    String.format(Locale.US, "%.2f", (double) deltaTime / 1000.01) + "," +
                    String.format(Locale.US, "%.2f", startHeading) + "," +
                    String.format(Locale.US, "%.2f", startPoseX) + "," +
                    String.format(Locale.US, "%.2f", startPoseY) + "," +
                    String.format(Locale.US, "%.2f", endHeading) + "," +
                    String.format(Locale.US, "%.2f", endPoseX) + "," +
                    String.format(Locale.US, "%.2f", endPoseY) + "," +
                    String.format(Locale.US, "%.2f", startHeading - endHeading) + "," +
                    String.format(Locale.US, "%.2f", startPoseX - endPoseX) + "," +
                    String.format(Locale.US, "%.2f", startPoseY - endPoseY);
            log( message );
        }
    }
}
