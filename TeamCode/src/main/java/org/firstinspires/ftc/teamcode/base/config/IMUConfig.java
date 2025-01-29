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

import androidx.annotation.NonNull;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot.UsbFacingDirection;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot.LogoFacingDirection;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.base.logging.RobotLogger;

import java.util.logging.Logger;

public class IMUConfig implements Validatable {
    public String              deviceName;
    public IMU                 imu;
    public IMU.Parameters      parameters;

    /**
     * Directions the logo of the ControlHub faces
     * Possible directions
     *  - UP
     *  - DOWN
     *  - FORWARD
     *  - BACKWARD
     *  - LEFT
     *  - RIGHT
     */
    public LogoFacingDirection logoFacingDirection;

    /**
     * Directions the USB port of the ControlHub faces
     * Possible directions
     *  - UP
     *  - DOWN
     *  - FORWARD
     *  - BACKWARD
     *  - LEFT
     *  - RIGHT
     */
    public UsbFacingDirection  usbFacingDirection;

    public void initialize(HardwareMap hardwareMap) {
        try {
            imu = hardwareMap.get(IMU.class, deviceName);
            parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                    logoFacingDirection,
                    usbFacingDirection));
            imu.initialize(parameters);
        } catch (Exception e) {
            Logger logger = RobotLogger.getInstance().getConfigLogger();
            logger.throwing("MotorConfig", "Initialize", e);
        }
    }

    public boolean isValid() {
        return true;
    }

    @NonNull
    @Override
    public String toString() {
        var sb = new StringBuilder();

        sb.append("IMUConfig\n");
        sb.append("  deviceName=").append(deviceName).append("\n");
        sb.append("  logoFacingDirection=").append(logoFacingDirection).append("\n");
        sb.append("  usbFacingDirection=").append(usbFacingDirection).append("\n");

        return sb.toString();
    }
}
