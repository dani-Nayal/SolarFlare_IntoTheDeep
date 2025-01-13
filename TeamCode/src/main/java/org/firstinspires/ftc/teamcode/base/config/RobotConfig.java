package org.firstinspires.ftc.teamcode.base.config;

import java.util.Arrays;
import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.PrintStream;
import java.util.Objects;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;
import org.json.JSONTokener;

import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;

public class RobotConfig {
    private static RobotConfig instance = null;
    JSONObject json = null;

    private RobotConfig(String robotName) {
        initialize(robotName);
    }

    private void initialize(String robotName) {
        String configFileName = robotName + ".json";
        try(InputStream input = Objects.requireNonNull(RobotConfig.class.getClassLoader())
                .getResourceAsStream(configFileName))
        {
            if (input == null)
                throw new IllegalArgumentException("No configuration available for " + robotName);

            BufferedReader reader = new BufferedReader(new InputStreamReader(input));
            StringBuilder  sb     = new StringBuilder();
            String         line;
            while((line = reader.readLine()) != null)
                sb.append(line);
            try {
                JSONTokener tokener = new JSONTokener(sb.toString());
                json                = new JSONObject(tokener);
            } catch (Exception e) {
                e.printStackTrace();
            }
        } catch (IOException ex) {
            ex.printStackTrace();
        }
    }

    public static RobotConfig createInstance(String robotName) {
        instance = new RobotConfig(robotName);
        return instance;
    }

    public static RobotConfig getInstance() {
        if(instance == null)
            throw new IllegalStateException("Config object not initialized");
        return instance;
    }

    public String[] getMotorNames() throws JSONException {
        // An alternative to
        // JSONObject.getNames(json.getJSONObject("Motors"));
        //
        JSONArray motorJSONNames = json.getJSONObject("Motors").names();
        assert motorJSONNames != null;
        String[]  motorNames     = new String[motorJSONNames.length()];
        for(int i=0; i<motorNames.length; i++)
            motorNames[i] = motorJSONNames.getString(i);
        return motorNames;
    }

    public MotorEnum[] getMotorEnums() throws JSONException {
        String[]    motorNames = getMotorNames();
        MotorEnum[] motorEnums = new MotorEnum[motorNames.length];
        for(int i=0; i<motorNames.length; i++) {
            motorEnums[i] = getMotorEnum(motorNames[i]);
        }
        return motorEnums;
    }

    public static MotorEnum getMotorEnum(String motorName) {
        return MotorEnum.valueOf(motorName);
    }

    public String getMotorString(String motorName, String propertyName) throws JSONException {
        return json.getJSONObject("Motors").getJSONObject(motorName).getString(propertyName);
    }

    public String getMotorString(MotorEnum motorEnum, String propertyName) throws JSONException {
        return getMotorString(motorEnum.toString(), propertyName);
    }

    public double getMotorDouble(String motorName, String propertyName) throws JSONException {
        return json.getJSONObject("Motors").getJSONObject(motorName).getDouble(propertyName);
    }

    public double getMotorDouble(MotorEnum motorEnum, String propertyName) throws JSONException {
        return getMotorDouble(motorEnum.toString(), propertyName);
    }

    public Direction getMotorDirection(MotorEnum motorEum) throws JSONException {
        return Direction.valueOf(getMotorString(motorEum, "direction"));
    }

    public RunMode getMotorRunMode(MotorEnum motorEum) throws JSONException {
        return RunMode.valueOf(getMotorString(motorEum, "runMode"));
    }
    public ZeroPowerBehavior getMotorZeroPowerBehavior(MotorEnum motorEum) throws JSONException {
        return ZeroPowerBehavior.valueOf(getMotorString(motorEum, "zeroPowerBehavior"));
    }

    public String[] getServoNames() throws JSONException {
        // An alternative to
        // JSONObject.getNames(json.getJSONObject("Servos"));
        //
        JSONArray servoJSONNames = json.getJSONObject("Servos").names();
        assert  servoJSONNames != null;
        String[]  servoNames     = new String[servoJSONNames.length()];
        for(int i=0; i<servoNames.length; i++)
            servoNames[i] = servoJSONNames.getString(i);
        return servoNames;
    }

    public ServoEnum[] getServoEnums() throws JSONException {
        String[]    servoNames    = getServoNames();
        ServoEnum[] servoEnums    = new ServoEnum[servoNames.length];
        for(int i=0; i<servoNames.length; i++) {
            servoEnums[i] = getServoEnum(servoNames[i]);
        }
        return servoEnums;
    }

    public static ServoEnum getServoEnum(String servoName) {
        return ServoEnum.valueOf(servoName);
    }

    public String getServoString(String servoName, String propertyName) throws JSONException {
        return json.getJSONObject("Servos").getJSONObject(servoName).getString(propertyName);
    }

    public String getServoString(ServoEnum servoEnum, String propertyName) throws JSONException {
        return getServoString(servoEnum.toString(), propertyName);
    }

    public double getServoDouble(String servoName, String propertyName) throws JSONException {
        return json.getJSONObject("Servos").getJSONObject(servoName).getDouble(propertyName);
    }

    public double getServoDouble(ServoEnum servoEnum, String propertyName) throws JSONException {
        return getServoDouble(servoEnum.toString(), propertyName);
    }

    public String getIMUString(String propertyName) throws JSONException {
        return json.getJSONObject("IMU").getString(propertyName);
    }

    public double getIMUDouble(String propertyName) throws JSONException {
        return json.getJSONObject("IMU").getDouble(propertyName);
    }

    public String getPinPointString(String propertyName) throws JSONException {
        return json.getJSONObject("PinPoint").getString(propertyName);
    }

    public double getPinPointDouble(String propertyName) throws JSONException {
        return json.getJSONObject("PinPoint").getDouble(propertyName);
    }

    public String getLimeLightString(String propertyName) throws JSONException {
        return json.getJSONObject("LimeLight").getString(propertyName);
    }

    public double getLimeLightDouble(String propertyName) throws JSONException {
        return json.getJSONObject("LimeLight").getDouble(propertyName);
    }

    public static void main(String[] args) {
        try {
            PrintStream out = System.out;
            RobotConfig config = RobotConfig.createInstance("RobotConfig");

            // Motor names, Enums
            String[]    motorNames = config.getMotorNames();
            out.println("MotorNames: " + Arrays.toString(motorNames));
            MotorEnum[] motorEnums = config.getMotorEnums();
            out.println("MotorEnums: " + Arrays.toString(motorEnums));
            for (MotorEnum motorEnum : motorEnums) {
                out.println("Motor("+motorEnum+").direction="+config.getMotorString(motorEnum,"direction"));
                out.println("Motor("+motorEnum+").direction="+config.getMotorDirection(motorEnum));
                out.println("Motor("+motorEnum+").kP="+config.getMotorDouble(motorEnum,"kP"));
                out.println("Motor("+motorEnum+").kI="+config.getMotorDouble(motorEnum,"kI"));
                out.println("Motor("+motorEnum+").kD="+config.getMotorDouble(motorEnum,"kD"));
                out.println("Motor("+motorEnum+").runMode="+config.getMotorRunMode(motorEnum));
                out.println("Motor("+motorEnum+").zeroPowerBehavior="+config.getMotorZeroPowerBehavior(motorEnum));
            }

            // Servo names, Enums
            String[]    servoNames = config.getServoNames();
            out.println("ServoNames: " + Arrays.toString(servoNames));
            ServoEnum[] servoEnums = config.getServoEnums();
            out.println("MotorEnums: " + Arrays.toString(servoEnums));
            for (ServoEnum servoEnum : servoEnums) {
                out.println("Servo(" + servoEnum + ").direction=" + config.getServoString(servoEnum, "direction"));
                out.println("Servo(" + servoEnum + ").minServoPosition=" + config.getServoDouble(servoEnum, "minServoPosition"));
                out.println("Servo(" + servoEnum + ").maxServoPosition=" + config.getServoDouble(servoEnum, "maxServoPosition"));
                out.println("Servo(" + servoEnum + ").degreesPerSecond=" + config.getServoDouble(servoEnum, "degreesPerSecond"));
            }
            out.println("IMU.deviceName=" + config.getIMUString("deviceName"));
            out.println("IMU.testVariable=" + config.getIMUDouble("testVariable"));
            out.println("PinPoint.deviceName=" + config.getPinPointString("deviceName"));
            out.println("PinPoint.testVariable=" + config.getPinPointDouble("testVariable"));
            out.println("LimeLight.deviceName=" + config.getLimeLightString("deviceName"));
            out.println("LimeLight.pollingRate=" + config.getLimeLightDouble("pollingRate"));
        } catch (Exception e) {
            e.printStackTrace();
        }
    }
}
