package odom;

public class Odom {
    private final double forwardTrackerCenterDistance;
    private final double sidewaysTrackerCenterDistance;

    private double forwardTrackerPosition;
    private double sidewaysTrackerPosition;

    public double xPosition;
    public double yPosition;
    public double orientationDeg;

    public Odom(double forwardTrackerCenterDistance, double sidewaysTrackerCenterDistance) {
        this.forwardTrackerCenterDistance = forwardTrackerCenterDistance;
        this.sidewaysTrackerCenterDistance = sidewaysTrackerCenterDistance;
    }

    public void setPosition(double x, double y, double orientation) {
        this.xPosition = x;
        this.yPosition = y;
        this.orientationDeg = orientation;
        this.forwardTrackerPosition  = 0;
        this.sidewaysTrackerPosition = 0;
    }

    public void updatePosition(double forwardTrackerPosition, double sidewaysTrackerPosition, double orientationDeg) {
        double forwardDelta = forwardTrackerPosition - this.forwardTrackerPosition;
        double sidewaysDelta = sidewaysTrackerPosition - this.sidewaysTrackerPosition;

        this.forwardTrackerPosition = forwardTrackerPosition;
        this.sidewaysTrackerPosition = sidewaysTrackerPosition;

        double orientationRad = Math.toRadians(orientationDeg);
        double prevOrientationRad = Math.toRadians(this.orientationDeg);
        double orientationDeltaRad = orientationRad - prevOrientationRad;

        this.orientationDeg = orientationDeg;
        // TODO: write explanation for math later
        double localX, localY;
        if (orientationDeltaRad == 0) {
            localX = sidewaysDelta;
            localY = forwardDelta;
        } else {
            localX = (2* Math.sin(orientationDeltaRad / 2))
                    * ((sidewaysDelta / orientationDeltaRad) + sidewaysTrackerCenterDistance);
            localY = (2* Math.sin(orientationDeltaRad /2))
                    * ((forwardDelta / orientationDeltaRad) + forwardTrackerCenterDistance);
        }
        double localPolarAngle;
        double localPolarLength;
        if (localX == 0 && localY == 0) {
            localPolarAngle  = 0;
            localPolarLength = 0;
        } else {
            localPolarAngle  = Math.atan2(localY, localX);
            localPolarLength = Math.sqrt(localX * localX + localY * localY);
        }

        double globalPolarAngle = localPolarAngle - prevOrientationRad - (orientationDeltaRad / 2);

        this.xPosition += localPolarLength * Math.cos(globalPolarAngle);
        this.yPosition += localPolarLength * Math.sin(globalPolarAngle);
    }
}
 