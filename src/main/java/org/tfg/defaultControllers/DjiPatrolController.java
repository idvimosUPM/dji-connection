package org.tfg.defaultControllers;

import com.cyberbotics.webots.controller.Camera;
import com.cyberbotics.webots.controller.GPS;
import com.cyberbotics.webots.controller.Gyro;
import com.cyberbotics.webots.controller.InertialUnit;
import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.Robot;

public class DjiPatrolController extends Robot {
    // Constants, empirically found
    private static final double K_VERTICAL_THRUST = 68.5; // With this thrust, the drone lifts
    private static final double K_VERTICAL_OFFSET = 0.6; // Vertical offset where the drone stabilizes
    private static final double K_VERTICAL_P = 3.0; // P constant of the vertical PID
    private static final double K_ROLL_P = 50.0; // P constant of the roll PID
    private static final double K_PITCH_P = 30.0; // P constant of the pitch PID
    private static final double MAX_YAW_DISTURBANCE = 0.4; // Maximum yaw disturbance
    private static final double MAX_PITCH_DISTURBANCE = -1; // Maximum pitch disturbance
    private static final double TARGET_PRECISION = 0.5; // Precision between the target position and the drone position in meters

    // Instance variables
    private final int timeStep;
    private Camera camera;
    private final InertialUnit imu;
    private final GPS gps;
    private final Gyro gyro;
    private final Motor frontLeftMotor;
    private final Motor frontRightMotor;
    private final Motor rearLeftMotor;
    private final Motor rearRightMotor;
    private Motor cameraPitchMotor;
    private final double[] currentPose = new double[6]; // X, Y, Z, yaw, pitch, roll
    private final double[] targetPosition = new double[3];
    private int targetIndex = 0;
    private double targetAltitude = 0;

    // Constructor
    public DjiPatrolController() {
        this.timeStep = (int) getBasicTimeStep();

        // Get and enable devices
        this.camera = getCamera("camera");
        this.camera.enable(this.timeStep);
        this.imu = getInertialUnit("inertial unit");
        this.imu.enable(this.timeStep);
        this.gps = getGPS("gps");
        this.gps.enable(this.timeStep);
        this.gyro = getGyro("gyro");
        this.gyro.enable(this.timeStep);

        // Initialize motors
        this.frontLeftMotor = getMotor("front left propeller");
        this.frontRightMotor = getMotor("front right propeller");
        this.rearLeftMotor = getMotor("rear left propeller");
        this.rearRightMotor = getMotor("rear right propeller");
        this.cameraPitchMotor = getMotor("camera pitch");
        this.cameraPitchMotor.setPosition(0.7);

        Motor[] motors = {this.frontLeftMotor, this.frontRightMotor, this.rearLeftMotor, this.rearRightMotor};
        for (Motor motor : motors) {
            motor.setPosition(Double.POSITIVE_INFINITY);
            motor.setVelocity(1);
        }
    }

    // Method to clamp a value between a minimum and a maximum
    private double clamp(double value, double min, double max) {
        return Math.min(Math.max(value, min), max);
    }

    // Method to set the current position of the drone
    private void setPosition(double[] pos) {
        System.arraycopy(pos, 0, this.currentPose, 0, pos.length);
    }

    // Method to move the drone to the target
    private double[] moveToTarget(double[][] waypoints, boolean verboseMovement, boolean verboseTarget) {
        if (this.targetPosition[0] == 0 && this.targetPosition[1] == 0) { // Initialization
            this.targetPosition[0] = waypoints[0][0];
            this.targetPosition[1] = waypoints[0][1];
            if (verboseTarget) {
                System.out.println("First target: " + this.targetPosition[0] + ", " + this.targetPosition[1]);
            }
        }

        // If the drone is at the position with a precision of TARGET_PRECISION
        if (Math.abs(this.targetPosition[0] - this.currentPose[0]) < TARGET_PRECISION &&
                Math.abs(this.targetPosition[1] - this.currentPose[1]) < TARGET_PRECISION) {
            this.targetIndex++;
            if (this.targetIndex >= waypoints.length) {
                this.targetIndex = 0;
            }
            this.targetPosition[0] = waypoints[this.targetIndex][0];
            this.targetPosition[1] = waypoints[this.targetIndex][1];
            if (verboseTarget) {
                System.out.println("Target reached! New target: " + this.targetPosition[0] + ", " + this.targetPosition[1]);
            }
        }

        // This will be in ]-pi;pi]
        this.targetPosition[2] = Math.atan2(this.targetPosition[1] - this.currentPose[1], this.targetPosition[0] - this.currentPose[0]);
        // This is now in ]-2pi;2pi[
        double angleLeft = this.targetPosition[2] - this.currentPose[5];
        // Normalize turn angle to ]-pi;pi]
        angleLeft = (angleLeft + 2 * Math.PI) % (2 * Math.PI);
        if (angleLeft > Math.PI) {
            angleLeft -= 2 * Math.PI;
        }

        // Turn the drone to the left or to the right according to the value and the sign of angleLeft
        double yawDisturbance = MAX_YAW_DISTURBANCE * angleLeft / (2 * Math.PI);
        // Non-proportional and decreasing function
        double pitchDisturbance = clamp(Math.log10(Math.abs(angleLeft)), MAX_PITCH_DISTURBANCE, 0.1);

        if (verboseMovement) {
            double distanceLeft = Math.sqrt(Math.pow(this.targetPosition[0] - this.currentPose[0], 2) +
                    Math.pow(this.targetPosition[1] - this.currentPose[1], 2));
            System.out.println("Remaining angle: " + angleLeft + ", remaining distance: " + distanceLeft);
        }

        return new double[]{yawDisturbance, pitchDisturbance};
    }

    // Main method to run the drone
    public void run() {
        double t1 = getTime();

        double rollDisturbance = 0;
        double pitchDisturbance = 0;
        double yawDisturbance = 0;

        // Specify the patrol coordinates
        double[][] waypoints = {{-30, 20}, {-60, 20}, {-60, 10}, {-30, 5}};
        // Target altitude of the drone in meters
        this.targetAltitude = 15;

        while (step(this.timeStep) != -1) {
            // Read sensors
            double[] imuValues = this.imu.getRollPitchYaw();
            double[] gpsValues = this.gps.getValues();
            double[] gyroValues = this.gyro.getValues();
            setPosition(new double[]{gpsValues[0], gpsValues[1], gpsValues[2], imuValues[0], imuValues[1], imuValues[2]});

            if (gpsValues[2] > this.targetAltitude - 1) {
                // As soon as it reaches the target altitude, compute the disturbances to go to the given waypoints
                if (getTime() - t1 > 0.1) {
                    double[] disturbances = moveToTarget(waypoints, true, true);
                    yawDisturbance = disturbances[0];
                    pitchDisturbance = disturbances[1];
                    t1 = getTime();
                }
            }

            double rollInput = K_ROLL_P * clamp(imuValues[0], -1, 1) + gyroValues[0] + rollDisturbance;
            double pitchInput = K_PITCH_P * clamp(imuValues[1], -1, 1) + gyroValues[1] + pitchDisturbance;
            double yawInput = yawDisturbance;
            double clampedDifferenceAltitude = clamp(this.targetAltitude - gpsValues[2] + K_VERTICAL_OFFSET, -1, 1);
            double verticalInput = K_VERTICAL_P * Math.pow(clampedDifferenceAltitude, 3.0);

            double frontLeftMotorInput = K_VERTICAL_THRUST + verticalInput - yawInput + pitchInput - rollInput;
            double frontRightMotorInput = K_VERTICAL_THRUST + verticalInput + yawInput + pitchInput + rollInput;
            double rearLeftMotorInput = K_VERTICAL_THRUST + verticalInput + yawInput - pitchInput - rollInput;
            double rearRightMotorInput = K_VERTICAL_THRUST + verticalInput - yawInput - pitchInput + rollInput;

            this.frontLeftMotor.setVelocity(frontLeftMotorInput);
            this.frontRightMotor.setVelocity(-frontRightMotorInput);
            this.rearLeftMotor.setVelocity(-rearLeftMotorInput);
            this.rearRightMotor.setVelocity(rearRightMotorInput);
        }
    }

    // Main method
    public static void main(String[] args) {
        DjiPatrolController robot = new DjiPatrolController();
        robot.run();
    }
}