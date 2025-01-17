package org.firstinspires.ftc.teamcode.base.config;

import java.util.Arrays;
import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.PrintStream;
import java.util.logging.Logger;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;
import org.json.JSONTokener;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot.LogoFacingDirection;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot.UsbFacingDirection;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;


public class RobotConfig {
    private static RobotConfig instance = null;
    public  static Logger      logger;

    private        JSONObject  json     = null;

    static {
        logger = RobotLogger.getConfigLogger();
    }

    private RobotConfig(String robotName) {
        initialize(robotName);
    }

    private void initialize(String robotName) {
        logger.entering("RobotConfig", "initialize", robotName);
        String configFileName = robotName + ".json";
        try(InputStream input = RobotConfig.class.getResourceAsStream(configFileName))
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
                logger.throwing("RobotConfig", "initialize", e );
            }
        } catch (IOException ex) {
            logger.throwing("RobotConfig", "initialize", ex);
        }
        logger.exiting("RobotConfig", "initialize");
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

    public RobotDimensions getRobotDimensions()
            throws JSONException {
        logger.entering("RobotConfig", "getRobotDimensions");
        JSONObject jsonDimensions       = json.getJSONObject("robotDimensions");
        double     length               = jsonDimensions.getDouble("length");
        double     width                = jsonDimensions.getDouble("width");
        RobotDimensions robotDimensions = new RobotDimensions(length, width);
        logger.exiting("RobotConfig", "getRobotDimensions", robotDimensions);
        return robotDimensions;
    }

    public String getCalibrationString(String propertyName) {
        logger.entering("RobotConfig", "getCalibrationString", propertyName);
        String calibrationString;
        try {
            calibrationString = json.getJSONObject("Calibration").getString(propertyName);
        } catch(JSONException e) {
            logger.throwing("RobotConfig", "getCalibrationString", e);
            return null;
        }
        logger.exiting("RobotConfig", "getCalibrationString", calibrationString);
        return calibrationString;
    }

    public Double getCalibrationDouble(String propertyName) {
        logger.entering("RobotConfig", "getCalibrationDouble", propertyName);
        Double pinpointDouble;
        try {
            pinpointDouble = json.getJSONObject("Pinpoint").getDouble(propertyName);
        } catch(JSONException e) {
            logger.throwing("RobotConfig", "getCalibrationDouble", e);
            return null;
        }
        logger.exiting("RobotConfig", "getCalibrationDouble", pinpointDouble);
        return pinpointDouble;
    }

    public Integer getCalibrationInt(String propertyName) {
        logger.entering("RobotConfig", "getCalibrationInt", propertyName);
        Integer pinpointInt;
        try {
            pinpointInt = json.getJSONObject("Pinpoint").getInt(propertyName);
        } catch(JSONException e) {
            logger.throwing("RobotConfig", "getCalibrationInt", e);
            return null;
        }
        logger.exiting("RobotConfig", "getPinpointInt", pinpointInt);
        return pinpointInt;
    }

    public String[] getMotorNames() throws JSONException {
        // An alternative to
        // JSONObject.getNames(json.getJSONObject("Motors"));
        //
        logger.entering("RobotConfig", "getMotorNames");
        JSONArray motorJSONNames = json.getJSONObject("Motors").names();
        if(motorJSONNames == null)
            return new String[] {};
        String[]  motorNames     = new String[motorJSONNames.length()];
        for(int i=0; i<motorNames.length; i++)
            motorNames[i] = motorJSONNames.getString(i);
        logger.exiting("RobotConfig", "getMotorNames", motorNames);
        return motorNames;
    }

    public MotorEnum[] getMotorEnums() throws JSONException {
        logger.entering("RobotConfig", "getMotorEnums");
        String[]    motorNames = getMotorNames();
        MotorEnum[] motorEnums = new MotorEnum[motorNames.length];
        for(int i=0; i<motorNames.length; i++) {
            motorEnums[i] = getMotorEnum(motorNames[i]);
        }
        logger.exiting("RobotConfig", "getMotorEnums", motorEnums);
        return motorEnums;
    }

    public static MotorEnum getMotorEnum(String motorName) {
        logger.entering("RobotConfig", "getMotorEnum", motorName);
        MotorEnum motorEnum = MotorEnum.valueOf(motorName);
        logger.exiting("RobotConfig", "getMotorEnum", motorEnum);
        return motorEnum;
    }

    public String getMotorString(String motorName, String propertyName)
            throws JSONException {
        return json.getJSONObject("Motors").getJSONObject(motorName).getString(propertyName);
    }

    public String getMotorString(MotorEnum motorEnum, String propertyName)
            throws JSONException {
        logger.entering("RobotConfig",
                "getMotorString",
                new String[] {motorEnum.name(), propertyName});
        String motorString = getMotorString(motorEnum.toString(), propertyName);
        logger.exiting("RobotConfig", "getMotorString", motorString);
        return motorString;
    }

    public double getMotorDouble(String motorName, String propertyName)
            throws JSONException {
        return json.getJSONObject("Motors").getJSONObject(motorName).getDouble(propertyName);
    }

    public double getMotorDouble(MotorEnum motorEnum, String propertyName)
            throws JSONException {
        logger.entering("RobotConfig",
                "getMotorDouble",
                new String[] {motorEnum.name(), propertyName});
        double motorDouble = getMotorDouble(motorEnum.toString(), propertyName);
        logger.exiting("RobotConfig", "getMotorDouble", motorDouble);
        return motorDouble;
    }

    public int getMotorInt(String motorName, String propertyName)
            throws JSONException {
        return json.getJSONObject("Motors").getJSONObject(motorName).getInt(propertyName);
    }

    public int getMotorInt(MotorEnum motorEnum, String propertyName)
            throws JSONException {
        logger.entering("RobotConfig",
                "getMotorInt",
                new String[] {motorEnum.name(), propertyName});
        int motorInt = getMotorInt(motorEnum.toString(), propertyName);
        logger.exiting("RobotConfig", "getMotorInt", motorInt);
        return motorInt;
    }

    public DcMotorSimple.Direction getMotorDirection(MotorEnum motorEum)
            throws JSONException {
        logger.entering("RobotConfig", "getMotorDirection", motorEum);
        DcMotorSimple.Direction direction = DcMotorSimple.Direction.valueOf(getMotorString(motorEum, "direction"));
        logger.exiting("RobotConfig", "getMotorDirection", direction);
        return direction;
    }

    public RunMode getMotorRunMode(MotorEnum motorEum)
            throws JSONException {
        logger.entering("RobotConfig", "getMotorRunMode", motorEum);
        RunMode motorRunMode = RunMode.valueOf(getMotorString(motorEum, "runMode"));
        logger.exiting("RobotConfig", "getMotorRunMode", motorRunMode);
        return motorRunMode;
    }

    public ZeroPowerBehavior getMotorZeroPowerBehavior(MotorEnum motorEum)
            throws JSONException {
        logger.entering("RobotConfig", "getMotorZeroPowerBehavior", motorEum);
        ZeroPowerBehavior zpb = ZeroPowerBehavior.valueOf(getMotorString(motorEum, "zeroPowerBehavior"));
        logger.exiting("RobotConfig", "getMotorZeroPowerBehavior", zpb);
        return zpb;
    }

    public String[] getServoNames()
            throws JSONException {
        // An alternative to
        // JSONObject.getNames(json.getJSONObject("Servos"));
        //
        logger.entering("RobotConfig", "getServoNames");
        JSONArray servoJSONNames = json.getJSONObject("Servos").names();
        if(servoJSONNames == null)
            return new String[] {};
        String[]  servoNames     = new String[servoJSONNames.length()];
        for(int i=0; i<servoNames.length; i++)
            servoNames[i] = servoJSONNames.getString(i);
        logger.exiting("RobotConfig", "getServoNames", servoJSONNames);
        return servoNames;
    }

    public ServoEnum[] getServoEnums() throws JSONException {
        logger.entering("RobotConfig", "getServoEnums");
        String[]    servoNames    = getServoNames();
        ServoEnum[] servoEnums    = new ServoEnum[servoNames.length];
        for(int i=0; i<servoNames.length; i++) {
            servoEnums[i] = getServoEnum(servoNames[i]);
        }
        logger.exiting("RobotConfig", "getServoEnums", servoEnums);
        return servoEnums;
    }

    public static ServoEnum getServoEnum(String servoName) {
        logger.entering("RobotConfig", "getServoEnum", servoName);
        ServoEnum servoEnum = ServoEnum.valueOf(servoName);
        logger.exiting("RobotConfig", "getServoEnum", servoEnum);
        return servoEnum;
    }

    public String getServoString(String servoName, String propertyName)
            throws JSONException {
        return json.getJSONObject("Servos").getJSONObject(servoName).getString(propertyName);
    }

    public String getServoString(ServoEnum servoEnum, String propertyName)
            throws JSONException {
        logger.entering("RobotConfig",
                "getServoString",
                new String[] {servoEnum.name(), propertyName});
        String servoString = getServoString(servoEnum.toString(), propertyName);
        logger.exiting("RobotConfig", "getServoString", servoString);
        return servoString;
    }

    public double getServoDouble(String servoName, String propertyName)
            throws JSONException {
        return json.getJSONObject("Servos").getJSONObject(servoName).getDouble(propertyName);
    }

    public double getServoDouble(ServoEnum servoEnum, String propertyName)
            throws JSONException {
        logger.entering("RobotConfig",
                "getServoDouble",
                new String[] {servoEnum.name(), propertyName});
        double servoDouble = getServoDouble(servoEnum.toString(), propertyName);
        logger.exiting("RobotConfig", "getServoDouble", servoDouble);
        return servoDouble;
    }

    public int getServoInt(String servoName, String propertyName)
            throws JSONException {
        return json.getJSONObject("Servos").getJSONObject(servoName).getInt(propertyName);
    }

    public int getServoInt(ServoEnum servoEnum, String propertyName)
            throws JSONException {
        logger.entering("RobotConfig",
                "getServoInt",
                new String[] {servoEnum.name(), propertyName});
        int servoInt = getServoInt(servoEnum.toString(), propertyName);
        logger.exiting("RobotConfig", "getServoInt", servoInt);
        return servoInt;
    }

    public Servo.Direction getServoDirection(ServoEnum servoEnum)
            throws JSONException {
        logger.entering("RobotConfig", "getServoDirection");
        Servo.Direction direction = Servo.Direction.valueOf(getServoString(servoEnum, "direction"));
        logger.exiting("RobotConfig", "getServoDirection", direction);
        return direction;
    }

    public String getIMUString(String propertyName)
            throws JSONException {
        logger.entering("RobotConfig", "getIMUString", propertyName);
        String imuString = json.getJSONObject("IMU").getString(propertyName);
        logger.exiting("RobotConfig", "getIMUString", imuString);
        return imuString;
    }

    public double getIMUDouble(String propertyName)
            throws JSONException {
        logger.entering("RobotConfig", "getIMUDouble", propertyName);
        double imuDouble = json.getJSONObject("IMU").getDouble(propertyName);
        logger.exiting("RobotConfig", "getIMUDouble", imuDouble);
        return imuDouble;
    }

    public int getIMUInt(String propertyName)
            throws JSONException {
        logger.entering("RobotConfig", "getIMUDouble", propertyName);
        int imuInt = json.getJSONObject("IMU").getInt(propertyName);
        logger.exiting("RobotConfig", "getIMUInt", imuInt);
        return imuInt;
    }

    /**
     * Returns the direction the REV Control Hub Logo is facing on the robot
     * @return Possible values are:
     *  - UP
     *  - DOWN
     *  - FORWARD
     *  - BACKWARD
     *  - LEFT
     *  - RIGHT
     */
    public LogoFacingDirection getIMULogoFacingDirection()
            throws JSONException {
        logger.entering("RobotConfig", "getIMULogoFacingDirection");
        LogoFacingDirection lfd = LogoFacingDirection.valueOf(getIMUString("LogoFacingDirection"));
        logger.exiting("RobotConfig", "getIMULogoFacingDirection", lfd);
        return lfd;
    }

    /**
     * Returns the direction the REV Control Hub USB is facing on the robot
     * @return Possible values are:
     *  - UP
     *  - DOWN
     *  - FORWARD
     *  - BACKWARD
     *  - LEFT
     *  - RIGHT
     */
    public UsbFacingDirection getIMULUSBFacingDirection()
            throws JSONException {
        logger.entering("RobotConfig", "getIMUUSBFacingDirection");
        UsbFacingDirection ufd = UsbFacingDirection.valueOf(getIMUString("UsbFacingDirection"));
        logger.exiting("RobotConfig", "getIMUUSBFacingDirection", ufd);
        return ufd;
    }

    public String getPinpointString(String propertyName) {
        logger.entering("RobotConfig", "getPinpointString", propertyName);
        String pinpointString;
        try {
            pinpointString = json.getJSONObject("Pinpoint").getString(propertyName);
        } catch(JSONException e) {
            logger.throwing("RobotConfig", "getPinpointString", e);
            return null;
        }
        logger.exiting("RobotConfig", "getPinpointString", pinpointString);
        return pinpointString;
    }

    public Double getPinpointDouble(String propertyName) {
        logger.entering("RobotConfig", "getPinpointDouble", propertyName);
        Double pinpointDouble;
        try {
            pinpointDouble = json.getJSONObject("Pinpoint").getDouble(propertyName);
        } catch(JSONException e) {
            logger.throwing("RobotConfig", "getPinpointDouble", e);
            return null;
        }
        logger.exiting("RobotConfig", "getPinpointDouble", pinpointDouble);
        return pinpointDouble;
    }

    public Integer getPinpointInt(String propertyName) {
        logger.entering("RobotConfig", "getPinpointInt", propertyName);
        Integer pinpointInt;
        try {
            pinpointInt = json.getJSONObject("Pinpoint").getInt(propertyName);
        } catch(JSONException e) {
            logger.throwing("RobotConfig", "getPinpointInt", e);
            return null;
        }
        logger.exiting("RobotConfig", "getPinpointInt", pinpointInt);
        return pinpointInt;
    }

    public String getLimelightString(String propertyName) {
        logger.entering("RobotConfig", "getLimelightString", propertyName);
        String limelightString;
        try {
            limelightString = json.getJSONObject("Limelight").getString(propertyName);
        } catch (JSONException e) {
            logger.throwing("RobotConfig", "getLimelightString", e);
            return null;
        }
        logger.exiting("RobotConfig", "getLimelightString", limelightString);
        return limelightString;
    }

    public Double getLimelightDouble(String propertyName) {
        logger.entering("RobotConfig", "getLimelightDouble", propertyName);
        Double limelightDouble;
        try {
            limelightDouble = json.getJSONObject("Limelight").getDouble(propertyName);
        } catch(JSONException e) {
            logger.throwing("RobotConfig", "getLimelightDouble", e);
            return null;
        }
        logger.exiting("RobotConfig", "getLimelightDouble", limelightDouble);
        return limelightDouble;
    }

    public Integer getLimelightInt(String propertyName) {
        logger.entering("RobotConfig", "getLimelightInt", propertyName);
        Integer limelightInt;
        try {
            limelightInt = json.getJSONObject("Limelight").getInt(propertyName);
        } catch(JSONException e) {
            logger.throwing("RobotConfig", "getLimelightInt", e);
            return null;
        }
        logger.exiting("RobotConfig", "getLimelightInt", limelightInt);
        return limelightInt;
    }

    public static void main(String[] args) {
        try {
            PrintStream out = System.out;
            RobotConfig config = RobotConfig.createInstance("RobotConfig");

            // Robot Dimensions
            out.println("Robot Dimensions="+config.getRobotDimensions());
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
                out.println("Motor("+motorEnum+").minTarget="+config.getMotorInt(motorEnum,"minTarget"));
                out.println("Motor("+motorEnum+").maxTarget="+config.getMotorInt(motorEnum,"maxTarget"));
                out.println("Motor("+motorEnum+").maxAcceleration="+config.getMotorInt(motorEnum,"maxAcceleration"));
                out.println("Motor("+motorEnum+").maxVelocity="+config.getMotorInt(motorEnum,"maxVelocity"));
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
            out.println("IMU.LogoFacingDirection="+config.getIMULogoFacingDirection());
            out.println("IMU.USBFacingDirection="+config.getIMULUSBFacingDirection());
            out.println("IMU.testVariable=" + config.getIMUDouble("testVariable"));

            out.println("PinPoint.deviceName=" + config.getPinpointString("deviceName"));
            out.println("PinPoint.testVariable=" + config.getPinpointDouble("testVariable"));
            out.println("PinPoint.testVariable=" + config.getPinpointInt("testVariable"));

            out.println("Limelight.deviceName=" + config.getLimelightString("deviceName"));
            out.println("Limelight.pollingRate=" + config.getLimelightDouble("pollingRate"));
            out.println("Limelight.pollingRate=" + config.getLimelightInt("testVariable"));

        } catch (Exception e) {
            logger.throwing("RobotConfig", "main", e);
        }
    }
}
