package org.firstinspires.ftc.teamcode.base.config;

import java.util.logging.ConsoleHandler;
import java.util.logging.Level;
import java.util.logging.LogRecord;
import java.util.logging.Logger;

public class RobotLogger {
    public static Logger configLogger;

    static {
        ConsoleHandler handler = new ConsoleHandler() {
            @Override
            public synchronized void publish(final LogRecord record) {
                super.publish(record);
                flush();
            }
        };
        handler.se
        handler.setLevel(Level.ALL);

        configLogger = Logger.getLogger("robot.config");
        configLogger.setLevel(Level.ALL);
        configLogger.setUseParentHandlers(false);
        configLogger.addHandler(handler);
    }

    public static Logger getConfigLogger() {
        return configLogger;
    }
}
