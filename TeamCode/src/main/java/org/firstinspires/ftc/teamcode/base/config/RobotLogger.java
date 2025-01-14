package org.firstinspires.ftc.teamcode.base.config;

import java.util.logging.ConsoleHandler;
import java.util.logging.Level;
import java.util.logging.LogRecord;
import java.util.logging.Logger;
import java.util.logging.StreamHandler;

public class RobotLogger {
    private static Logger configLogger = null;

    public static Logger getConfigLogger() {
        if(configLogger == null) {
            ConsoleHandler handler = new ConsoleHandler() {
                @Override
                public synchronized void publish(final LogRecord record) {
                    super.publish(record);
                    flush();
                }
            };
            handler.setLevel(Level.ALL);

            configLogger = Logger.getLogger("robot.config");
            configLogger.setLevel(Level.ALL);
            configLogger.setUseParentHandlers(false);
            configLogger.addHandler(handler);
        }
        return configLogger;
    }
}
