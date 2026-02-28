package drive;

public class DriveUtil {
    public static double clamp(double value, double low, double high) {
        return Math.max(low, Math.min(high, value));
    }

    public static double clampMinVoltage(double value, double minVoltage) {
        if (Math.abs(value) > 0.001 && Math.abs(value) < minVoltage) {
            return Math.copySign(minVoltage, value);
        }
        return value;
    }

    public static double reduceNegative180to180(double degrees) {
        while (degrees >  180) degrees -= 360;
        while (degrees < -180) degrees += 360;
        return degrees;
    }

    public static double reduceNegative90to90(double degrees) {
        while (degrees >  90) degrees -= 180;
        while (degrees < -90) degrees += 180;
        return degrees;
    }

    public static boolean isLineSettled(double targetX, double targetY,
                                        double startAngle,
                                        double robotX, double robotY) {
        double dx = targetX - robotX;
        double dy = targetY - robotY;
        double angleToTarget = Math.toDegrees(Math.atan2(dx, dy));
        double diff = Math.abs(reduceNegative180to180(angleToTarget - startAngle));
        return diff > 90;
    }

    public static double leftVoltage(double driveOutput, double headingOutput) {
        return driveOutput - headingOutput;
    }

    public static double rightVoltage(double driveOutput, double headingOutput) {
        return driveOutput + headingOutput;
    }

    public static double bearing(double fromX, double fromY, double toX, double toY) {
        return Math.toDegrees(Math.atan2(toX - fromX, toY - fromY));
    }
}
