package org.firstinspires.ftc.teamcode.autonomous;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

public class CustomActions {
    public class SetExtendoTarget implements Action {
        private double target;
        public SetExtendoTarget(double target) {
            this.target = target;
        }
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            return false;
        }
    }
}
