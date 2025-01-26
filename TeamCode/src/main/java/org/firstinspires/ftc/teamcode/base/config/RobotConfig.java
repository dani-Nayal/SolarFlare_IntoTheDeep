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

import static org.firstinspires.ftc.teamcode.base.utils.JSONUtils.parseJSON;

import androidx.annotation.NonNull;

import java.io.InputStream;
import java.io.InputStreamReader;
import java.util.HashMap;
import java.util.logging.Logger;

public class RobotConfig {
    private static RobotConfig                     instance = null;
    public         String                          robotName;
    public         RobotDimensions                 robotDimensions;
    public         HashMap<MotorEnum, MotorConfig> motors;
    public         HashMap<ServoEnum, ServoConfig> servos;
    public         IMUConfig                       imu;
    public         PinpointConfig                  pinpoint;
    public         LimelightConfig                 limelight;

    public static RobotConfig createInstance(String robotName) {
        try(InputStream input = Application.getResourceAsStream(robotName + ".json")) {
            instance      = parseJSON(new InputStreamReader(input), RobotConfig.class);
        } catch(Exception e) {
            Logger logger = RobotLogger.getInstance().getConfigLogger();
            logger.throwing("RobotConfig", "createInstance", e);
        }
        return instance;
    }

    @NonNull
    @Override
    public String toString() {
        var sb = new StringBuilder();

        sb.append("RobotConfig\n");
        sb.append("robotName=")      .append(robotName)      .append("\n");
        sb.append("robotDimensions=").append(robotDimensions).append("\n");
        sb.append("motors\n");
        for(var entry: motors.entrySet())
            sb.append(entry.getKey().name())
                    .append("=\n")
                    .append(entry.getValue().toString())
                    .append("\n");
        sb.append("servos\n");
        for(var entry: servos.entrySet())
            sb.append(entry.getKey().name())
                    .append("=\n")
                    .append(entry.getValue().toString())
                    .append("\n");
        sb.append("imu\n")     .append(imu.toString())       .append("\n");
        sb.append("pinpoint\n").append(pinpoint.toString())  .append("\n");
        sb.append("limelight\n").append(limelight.toString()).append("\n");

        return sb.toString();
    }

    public static RobotConfig getInstance() {
        if(instance == null)
            throw new IllegalStateException("Config object not initialized");
        return instance;
    }

    public static void main(String[] args) {
        Logger   logger     = RobotLogger.getInstance().getConfigLogger();
        String[] robotNames = new String[] {"IntoTheDeep-V2", "Rig1Motor"};
        try {
            for(String robotName: robotNames) {
                RobotConfig config = RobotConfig.createInstance(robotName);
                System.out.println(config.toString());
            }
        } catch (Exception e) {
            System.out.println("RobotConfig.main throwing: " + e);
            logger.throwing("RobotConfig", "main", e);
        }
    }
}
