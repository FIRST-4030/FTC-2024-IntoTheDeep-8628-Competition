package org.firstinspires.ftc.teamcode;

import android.os.Environment;

import com.acmerobotics.roadrunner.Pose2d;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;

public class LogFile {
    private FileWriter logWriter;
    public String fullPath = null;

    long startTime;
    long deltaTime;
    String label = null;
    double startHeading = 0.0;
    double startPoseX   = 0.0;
    double startPoseY   = 0.0;
    double endHeading   = 0.0;
    double endPoseX     = 0.0;
    double endPoseY     = 0.0;

    public LogFile( String prefix, String fileType ) {

        String logFolder = Environment.getExternalStorageDirectory().getPath(); // /storage/emulated/0 also maps to /sdcard

        // Define a unique file name
        String timestamp = new SimpleDateFormat("yyyy-MM-dd HH:mm").format(new Date());
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
    }

    public void log(String message) {
        try {
            logWriter.write(message + "\n");
            logWriter.flush();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public void log(String type, double message) {
        try {
            logWriter.write(type + ": " + message + "\n");
            logWriter.flush();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public void logTitles() {
        String message;

        message = "," + "Delta" +
                "," + "Start" + "," + "Start" + "," + "Start" +
                "," + "End" + "," + "End" + "," + "End" +
                "," + "Delta" + "," + "Delta" + "," + "Delta";
        log( message );
        message = "," + "Time" +
                "," + "Heading" + "," + "X" + "," + "Y" +
                "," + "Heading" + "," + "X" + "," + "Y" +
                "," + "Heading" + "," + "X" + "," + "Y";
        log( message );
    }

    public void logSample( boolean isStart, String label, Pose2d pose ) {
        String message;

        if (isStart) {
            startTime    = System.currentTimeMillis();
            this.label   = label;
            startHeading = Math.toDegrees(pose.heading.toDouble());
            startPoseX   = pose.position.x;
            startPoseY   = pose.position.y;
        } else {
            deltaTime  = System.currentTimeMillis() - startTime;
            endHeading = Math.toDegrees(pose.heading.toDouble());
            endPoseX   = pose.position.x;
            endPoseY   = pose.position.y;

            message = this.label + "," +
                    String.format(Locale.US,"%.2f", (double)deltaTime/1000.01 ) + "," +
                    String.format(Locale.US,"%.2f", startHeading ) + "," +
                    String.format(Locale.US,"%.2f", startPoseX ) + "," +
                    String.format(Locale.US,"%.2f", startPoseY ) + "," +
                    String.format(Locale.US,"%.2f", endHeading ) + "," +
                    String.format(Locale.US,"%.2f", endPoseX ) + "," +
                    String.format(Locale.US,"%.2f", endPoseY ) + "," +
                    String.format(Locale.US,"%.2f", startHeading-endHeading ) + "," +
                    String.format(Locale.US,"%.2f", startPoseX-endPoseX ) + "," +
                    String.format(Locale.US,"%.2f", startPoseY-endPoseY );
            log( message );
        }
    }

    public void close() {
        try {
            if (logWriter != null) {
                logWriter.close();
            }
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}
