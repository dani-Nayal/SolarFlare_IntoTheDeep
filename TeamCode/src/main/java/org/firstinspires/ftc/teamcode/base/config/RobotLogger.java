/*
 * Copyright (c) 2025 Murad Nayal
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted
 * (subject to the limitations in the disclaimer below) provided that the following conditions are
 * met:
 *
 * Redistributions of source code must retain the above copyright notice, this list of conditions
 * and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions
 * and the following disclaimer in the documentation and/or other materials provided with the
 * distribution.
 *
 * Neither the name Murad Nayal nor the names of contributors to this material may be used to
 * endorse or promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS LICENSE. THIS
 * SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
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
        initializeMetricsFile("motion_profile",
                "%1$d,%2$d,%3$d,%4$10.5f,%5$10.5f,%6$10.5f%n",
                "iter",
                "targetMotorPosition",
                "motorPosition",
                "targetMotorPower",
                "motorPower",
                "motorVelocity"
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

    public RobotMetricsFile getMetricsFile(String tableName) {
        RobotMetricsFile file = null;
        try {
            file = Objects.requireNonNull(metricsFiles.get(tableName));
        } catch (NullPointerException e) {
            Logger.getGlobal().logp(
                    Level.SEVERE,
                    "RobotLogger",
                    "getMetricsFile",
                    "Table name: " + tableName + " not available",
                    e
            );
        }
        return file;
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
