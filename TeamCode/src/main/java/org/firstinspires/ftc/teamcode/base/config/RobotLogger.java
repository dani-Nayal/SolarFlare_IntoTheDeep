package org.firstinspires.ftc.teamcode.base.config;

import java.util.HashMap;
import java.util.Map;
import java.util.Objects;
import java.util.logging.ConsoleHandler;
import java.util.logging.Level;
import java.util.logging.LogRecord;
import java.util.logging.Logger;

public class RobotLogger {
    private static       RobotLogger                   instance;

    private        final Level                         configLoggerLevel = Level.ALL;
    private        final String                        configLoggerName  = "robot.config";
    private              Logger                        configLogger;
    private        final Map<String, RobotMetricsFile> metricsFiles = new HashMap<>();

    public static RobotLogger getInstance() {
        if(instance == null)
            instance = new RobotLogger();
        return instance;
    }

    private RobotLogger() {
        initializeConfigLogger();
        initializeMetricsFile("information",
                "%1$s,%2$d,%3$f%n",
                "variable1", "variable2", "variable3"
                );
    }

    private void initializeConfigLogger() {
        ConsoleHandler handler = new ConsoleHandler() {
            @Override
            public synchronized void publish(final LogRecord record) {
                super.publish(record);
                flush();
            }
        };
        handler.setLevel(configLoggerLevel);

        configLogger           = Logger.getLogger(configLoggerName);
        configLogger.setLevel(configLoggerLevel);
        configLogger.setUseParentHandlers(false);
        configLogger.addHandler(handler);
    }

    public Logger getConfigLogger() {
        return configLogger;
    }

    public void initializeMetricsFile(String tableName, String formatString, String... fieldNames) {
        RobotMetricsFile metricsFile = new RobotMetricsFile(tableName, formatString, fieldNames);
        if(metricsFile.isActive()) {
            metricsFiles.put(tableName, metricsFile);
        }
    }

    public void addData(String tableName, Object... data) {
        try {
            Objects.requireNonNull(metricsFiles.get(tableName)).addData(data);
        } catch (NullPointerException e) {
            Logger.getGlobal().logp(
                    Level.SEVERE,
                    "RobotLogger",
                    "addData",
                    "Table name: " + tableName + " not available",
                    e
            );
        }
    }
}
