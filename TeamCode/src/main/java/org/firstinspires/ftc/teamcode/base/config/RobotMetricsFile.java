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

import java.io.IOException;
import java.util.Formatter;
import java.util.logging.Level;
import java.util.logging.Logger;

public class RobotMetricsFile {
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

        String fullFileName = Application.getMetricsDirName() + "/" + tableName + ".csv";

        try {
            formatter = new Formatter(fullFileName);
        } catch (IOException e) {
            Logger.getGlobal().logp(
                    Level.SEVERE,
                    "RobotMetricsFile",
                    "open",
                    "Failed to open file:" + fullFileName,
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
