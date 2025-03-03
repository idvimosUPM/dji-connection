package org.tfg.custom;

import com.cyberbotics.webots.controller.Camera;
import com.cyberbotics.webots.controller.CameraRecognitionObject;
import com.cyberbotics.webots.controller.Compass;
import com.cyberbotics.webots.controller.GPS;
import com.cyberbotics.webots.controller.Gyro;
import com.cyberbotics.webots.controller.InertialUnit;
import com.cyberbotics.webots.controller.Keyboard;
import com.cyberbotics.webots.controller.LED;
import com.cyberbotics.webots.controller.Motor;

public class CustomDjiController extends CustomRobot {

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
    private double velocity = 1.0;

    public CustomDjiController() {
        camera = getCamera("camera");
        camera.enable(timeStep);
        camera.recognitionEnable(timeStep);
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
    }


    public void run() {

        initMotors(velocity);
        initKeyboard(timeStep);

        System.out.println("Start the drone...");

        waitBeforeStart();

        printInstructions();

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

            setItermitentFrontalLeds((int) time);

            cameraRollMotor.setPosition(-0.115 * rollVelocity);
            cameraPitchMotor.setPosition(-0.1 * pitchVelocity);

            int numberOfObjects = camera.getRecognitionNumberOfObjects();

            CameraRecognitionObject[] objects = camera.getRecognitionObjects();
            for (int i = 0; i < numberOfObjects; i++) {
                System.out.println("Model of object " + i + ": " + objects[i].getModel());
                System.out.println("Id of object " + i + ": " + objects[i].getId());
                System.out.println("Relative position of object " + i + ": " + objects[i].getPosition()[0] + " " +
                        objects[i].getPosition()[1] + " " + objects[i].getPosition()[2]);
                System.out.println("Relative orientation of object " + i + ": " + objects[i].getOrientation()[0] + " " +
                        objects[i].getOrientation()[1] + " " + objects[i].getOrientation()[2] + " " + objects[i].getOrientation()[3]);
                System.out.println("Size of object " + i + ": " + objects[i].getSize()[0] + " " + objects[i].getSize()[1]);
                System.out.println("Position of the object " + i + " on the camera image: " + objects[i].getPositionOnImage()[0] + " " +
                        objects[i].getPositionOnImage()[1]);
                System.out.println("Size of the object " + i + " on the camera image: " + objects[i].getSizeOnImage()[0] + " " +
                        objects[i].getSizeOnImage()[1]);
                for (int j = 0; j < objects[i].getNumberOfColors(); j++) {
                    int colorIndex = 3 * j;
                    if (colorIndex + 2 < objects[i].getColors().length) {
                        System.out.println("- Color " + (j + 1) + "/" + objects[i].getNumberOfColors() + ": " +
                                objects[i].getColors()[colorIndex] + " " + objects[i].getColors()[colorIndex + 1] + " " + objects[i].getColors()[colorIndex + 2]);
                    }
                }
            }

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

    private void setItermitentFrontalLeds(int time) {
        boolean ledState = time % 2 == 0;
        frontLeftLed.set(ledState ? 1 : 0);
        frontRightLed.set(ledState ? 0 : 1);
    }

    private static void printInstructions() {
        System.out.println("You can control the drone with your computer keyboard:");
        System.out.println("- 'up': move forward.");
        System.out.println("- 'down': move backward.");
        System.out.println("- 'right': turn right.");
        System.out.println("- 'left': turn left.");
        System.out.println("- 'shift + up': increase the target altitude.");
        System.out.println("- 'shift + down': decrease the target altitude.");
        System.out.println("- 'shift + right': strafe right.");
        System.out.println("- 'shift + left': strafe left.");
    }

    private void waitBeforeStart() {
        while (step(timeStep) != -1) {
            if (getTime() > 1.0)
                break;
        }
    }

    public static void main(String[] args) {
        CustomDjiController controller = new CustomDjiController();
        controller.run();
    }

    private double clamp(double value, double low, double high) {
        return Math.max(low, Math.min(value, high));
    }

    private void initKeyboard(int timeStep) {
        getKeyboard().enable(timeStep);
    }

    private void initMotors(double velocity) {
        this.velocity = velocity;
        Motor[] motors = {frontLeftMotor, frontRightMotor, rearLeftMotor, rearRightMotor};
        for (Motor motor : motors) {
            motor.setPosition(Double.POSITIVE_INFINITY);
            motor.setVelocity(velocity);
        }
    }
}
