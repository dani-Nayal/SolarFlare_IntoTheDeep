package org.firstinspires.ftc.teamcode.base.config;

import java.io.IOException;
import java.util.Formatter;
import java.util.logging.Level;
import java.util.logging.Logger;

public class RobotMetricsFile {
    @SuppressWarnings("SpellCheckingInspection")
    // Output directory from persistentDataPath on Android
    // Application.persistentDataPath points to /storage/emulated/<userid>/Android/data/<packagename>/files
    private static final String    dirName = "/storage/emulated/0/Android/data/com.qualcomm.ftcrobotcontroller/files/";
    private        final String    tableName;
    private              Formatter formatter;
    private        final String[]  fieldNames;
    private        final String    formatString;

    public RobotMetricsFile(String fileRoot, String formatString, String... fieldNames) {
        this.tableName    = fileRoot;
        this.formatString = formatString;
        this.fieldNames   = fieldNames;
        open();
    }

    public void open() {
        if(isActive())
            close();

        String fullFileName = dirName + "/" + tableName + ".csv";

        try {
            formatter = new Formatter(fullFileName);
        } catch (IOException e) {
            Logger.getGlobal().logp(
                    Level.SEVERE,
                    "RobotMetricsFile",
                    "open",
                    "Failed to open file" + fullFileName,
                    e);
            return;
        }
        formatter.format("%1$s", String.join(",", fieldNames) + "%n");
    }

    public void addData(Object... data) {
        if(!isActive()) {
            Logger.getGlobal().severe("RobotMetricsFile " + tableName + " not open. Skipping...");
            return;
        }
        formatter.format(formatString, data);
    }

    public boolean isActive() {
        return formatter != null;
    }

    public void close() {
        if(isActive()) {
            formatter.flush();
            formatter.close();
        }
    }
}
