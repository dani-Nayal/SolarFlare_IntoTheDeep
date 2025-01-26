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

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.json.JSONException;

public class MotorConfig{
    public MotorEnum                 motorEnum;
    public String                    deviceName;
    public DcMotorEx                 motor;
    public double                    kP;
    public double                    kI;
    public double                    kD;
    public DcMotor.RunMode           runMode;
    public DcMotorSimple.Direction   direction;
    public DcMotor.ZeroPowerBehavior zeroPowerBehavior;
    public int                       motorProfileResolution;
    public double                    maxPower;
    public int                       minTarget;
    public int                       maxTarget;
    public double                    maxAcceleration;
    public double                    maxVelocity;
    public MotorConfig(MotorEnum motorEnum, HardwareMap hardwareMap, RobotConfig robotConfig)
            throws JSONException {
        this.motorEnum         = motorEnum;
        this.deviceName        = robotConfig.getMotorString(motorEnum, "deviceName");
        this.motor             = hardwareMap.get(DcMotorEx.class, deviceName);
        this.kP                = robotConfig.getMotorDouble(motorEnum, "kP");
        this.kI                = robotConfig.getMotorDouble(motorEnum, "kI");
        this.kD                = robotConfig.getMotorDouble(motorEnum, "kD");
        this.runMode           = robotConfig.getMotorRunMode(motorEnum);
        this.direction         = robotConfig.getMotorDirection(motorEnum);
        this.zeroPowerBehavior = robotConfig.getMotorZeroPowerBehavior(motorEnum);
        this.maxPower          = robotConfig.getMotorInt(motorEnum, "maxPower");
        this.minTarget         = robotConfig.getMotorInt(motorEnum, "minTarget");
        this.maxTarget         = robotConfig.getMotorInt(motorEnum, "maxTarget");
        this.maxAcceleration   = robotConfig.getMotorInt(motorEnum, "maxAcceleration");
        this.maxVelocity       = robotConfig.getMotorInt(motorEnum, "maxVelocity");

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(this.runMode);
        motor.setDirection(this.direction);
        motor.setZeroPowerBehavior(this.zeroPowerBehavior);
    }
}
