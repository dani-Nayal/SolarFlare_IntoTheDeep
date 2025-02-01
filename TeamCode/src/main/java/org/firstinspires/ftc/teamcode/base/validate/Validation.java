package org.firstinspires.ftc.teamcode.base.validate;

import static java.lang.Double.isNaN;
import static java.lang.Double.NEGATIVE_INFINITY;
import static java.lang.Double.POSITIVE_INFINITY;

import org.firstinspires.ftc.teamcode.base.config.Validatable;
import org.firstinspires.ftc.teamcode.base.logging.RobotLogger;

import java.util.Locale;
import java.util.function.Predicate;
import java.util.logging.Level;
import java.util.logging.Logger;

public class Validation {
    private static Logger logger = RobotLogger.getInstance().getConfigLogger();

    private static String getFieldMessage(String fieldName, String message) {
        return String.format(Locale.US, "field=%1$s is: %2$s", fieldName, message);
    }

    private static void logSevere(String fieldName, String message) {
        logger.log(Level.SEVERE, getFieldMessage(fieldName, message));
    }

    public static boolean validate(String fieldName, Object field) {
        if(field == null) {
            logSevere(fieldName, "null");
            return false;
        } else if(field instanceof Validatable) {
            return ((Validatable) field).isValid();
        }
        return true;
    }

    public static <T> boolean validate(String fieldName, T field, Predicate<T> predicate) {
        if(!predicate.test(field)) {
            logSevere(fieldName, "Failed test");
            return false;
        } else {
            return true;
        }
    }

    public static boolean validate(String fieldName, Double field) {
        if(field==null || isNaN(field) || field==NEGATIVE_INFINITY || field==POSITIVE_INFINITY) {
            logSevere(fieldName, "is null, NaN or an infinity");
            return false;
        } else {
            return true;
        }
    }
}
