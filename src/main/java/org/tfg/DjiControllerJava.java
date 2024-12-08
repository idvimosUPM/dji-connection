package org.tfg;

import com.cyberbotics.webots.controller.*;

public class DjiControllerJava extends Robot {

    private final int timeStep = (int) getBasicTimeStep();
    private final Camera camera;
    private final LED frontLeftLed;
    private final LED frontRightLed;
    private final InertialUnit imu;
    private final GPS gps;
    private final Compass compass;
    private final Gyro gyro;
    private final Motor cameraRollMotor;
    private final Motor cameraPitchMotor;
    private final Motor frontLeftMotor;
    private final Motor frontRightMotor;
    private final Motor rearLeftMotor;
    private final Motor rearRightMotor;
    private double targetAltitude = 1.0;

    public DjiControllerJava() {
        camera = getCamera("camera");
        camera.enable(timeStep);
        frontLeftLed = getLED("front left led");
        frontRightLed = getLED("front right led");
        imu = getInertialUnit("inertial unit");
        imu.enable(timeStep);
        gps = getGPS("gps");
        gps.enable(timeStep);
        compass = getCompass("compass");
        compass.enable(timeStep);
        gyro = getGyro("gyro");
        gyro.enable(timeStep);
        cameraRollMotor = getMotor("camera roll");
        cameraPitchMotor = getMotor("camera pitch");
        frontLeftMotor = getMotor("front left propeller");
        frontRightMotor = getMotor("front right propeller");
        rearLeftMotor = getMotor("rear left propeller");
        rearRightMotor = getMotor("rear right propeller");

        Motor[] motors = {frontLeftMotor, frontRightMotor, rearLeftMotor, rearRightMotor};
        for (Motor motor : motors) {
            motor.setPosition(Double.POSITIVE_INFINITY);
            motor.setVelocity(1.0);
        }

        // Enable the keyboard
        getKeyboard().enable(timeStep);
    }

    public void run() {
        System.out.println("Start the drone...");

        while (step(timeStep) != -1) {
            if (getTime() > 1.0)
                break;
        }

        System.out.println("You can control the drone with your computer keyboard:");
        System.out.println("- 'up': move forward.");
        System.out.println("- 'down': move backward.");
        System.out.println("- 'right': turn right.");
        System.out.println("- 'left': turn left.");
        System.out.println("- 'shift + up': increase the target altitude.");
        System.out.println("- 'shift + down': decrease the target altitude.");
        System.out.println("- 'shift + right': strafe right.");
        System.out.println("- 'shift + left': strafe left.");

        final double kVerticalThrust = 68.5;
        final double kVerticalOffset = 0.6;
        final double kVerticalP = 3.0;
        final double kRollP = 50.0;
        final double kPitchP = 30.0;

        while (step(timeStep) != -1) {
            double time = getTime();

            double roll = imu.getRollPitchYaw()[0];
            double pitch = imu.getRollPitchYaw()[1];
            double altitude = gps.getValues()[2];
            double rollVelocity = gyro.getValues()[0];
            double pitchVelocity = gyro.getValues()[1];

            boolean ledState = ((int) time) % 2 == 0;
            frontLeftLed.set(ledState ? 1 : 0);
            frontRightLed.set(ledState ? 0 : 1);

            cameraRollMotor.setPosition(-0.115 * rollVelocity);
            cameraPitchMotor.setPosition(-0.1 * pitchVelocity);

            double rollDisturbance = 0.0;
            double pitchDisturbance = 0.0;
            double yawDisturbance = 0.0;
            int key = getKeyboard().getKey();
            while (key > 0) {
                switch (key) {
                    case Keyboard.UP:
                        pitchDisturbance = -2.0;
                        break;
                    case Keyboard.DOWN:
                        pitchDisturbance = 2.0;
                        break;
                    case Keyboard.RIGHT:
                        yawDisturbance = -1.3;
                        break;
                    case Keyboard.LEFT:
                        yawDisturbance = 1.3;
                        break;
                    case (Keyboard.SHIFT + Keyboard.RIGHT):
                        rollDisturbance = -1.0;
                        break;
                    case (Keyboard.SHIFT + Keyboard.LEFT):
                        rollDisturbance = 1.0;
                        break;
                    case (Keyboard.SHIFT + Keyboard.UP):
                        targetAltitude += 0.05;
                        System.out.println("target altitude: " + targetAltitude + " [m]");
                        break;
                    case (Keyboard.SHIFT + Keyboard.DOWN):
                        targetAltitude -= 0.05;
                        System.out.println("target altitude: " + targetAltitude + " [m]");
                        break;
                }
                key = getKeyboard().getKey();
            }

            double rollInput = kRollP * clamp(roll, -1.0, 1.0) + rollVelocity + rollDisturbance;
            double pitchInput = kPitchP * clamp(pitch, -1.0, 1.0) + pitchVelocity + pitchDisturbance;
            double yawInput = yawDisturbance;
            double clampedDifferenceAltitude = clamp(targetAltitude - altitude + kVerticalOffset, -1.0, 1.0);
            double verticalInput = kVerticalP * Math.pow(clampedDifferenceAltitude, 3.0);

            double frontLeftMotorInput = kVerticalThrust + verticalInput - rollInput + pitchInput - yawInput;
            double frontRightMotorInput = kVerticalThrust + verticalInput + rollInput + pitchInput + yawInput;
            double rearLeftMotorInput = kVerticalThrust + verticalInput - rollInput - pitchInput + yawInput;
            double rearRightMotorInput = kVerticalThrust + verticalInput + rollInput - pitchInput - yawInput;
            frontLeftMotor.setVelocity(frontLeftMotorInput);
            frontRightMotor.setVelocity(-frontRightMotorInput);
            rearLeftMotor.setVelocity(-rearLeftMotorInput);
            rearRightMotor.setVelocity(rearRightMotorInput);
        }
    }

    private double clamp(double value, double low, double high) {
        return Math.max(low, Math.min(value, high));
    }

    public static void main(String[] args) {
        DjiControllerJava controller = new DjiControllerJava();
        controller.run();
    }
}