package org.tfg.custom;

import com.cyberbotics.webots.controller.Camera;
import com.cyberbotics.webots.controller.Compass;
import com.cyberbotics.webots.controller.GPS;
import com.cyberbotics.webots.controller.Gyro;
import com.cyberbotics.webots.controller.InertialUnit;
import com.cyberbotics.webots.controller.Keyboard;
import com.cyberbotics.webots.controller.LED;
import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.Speaker;

import java.util.HashSet;
import java.util.Scanner;
import java.util.Set;

public class CustomDjiController extends CustomRobot {

    private Camera camera;
    private LED frontLeftLed;
    private LED frontRightLed;
    private InertialUnit imu;
    private GPS gps;
    private Compass compass;
    private Gyro gyro;
    private Motor cameraRollMotor;
    private Motor cameraPitchMotor;
    private Motor frontLeftMotor;
    private Motor frontRightMotor;
    private Motor rearLeftMotor;
    private Motor rearRightMotor;
    private Speaker speaker;

    private final int timeStep = (int) getBasicTimeStep();
    private double targetAltitude = 1.0;
    private double velocity = 1.0;

    private final Set<Integer> recognizedObjectIds = new HashSet<>();

    public CustomDjiController() {
        initializeComponents();
    }

    public void run() {

        initMotors(velocity);

        initKeyboard(timeStep);

        System.out.println("Start the drone...");

        waitBeforeStart();

        displaySearchOptions();
    }

    private void initControlByKeyboard() {
        final double kVerticalThrust = 68.5;
        final double kVerticalOffset = 0.6;
        final double kVerticalP = 3.0;
        final double kRollP = 50.0;
        final double kPitchP = 30.0;

        printInstructions();

        while (step(timeStep) != -1) {
            double time = getTime();

            double roll = imu.getRollPitchYaw()[0];
            double pitch = imu.getRollPitchYaw()[1];
            double altitude = gps.getValues()[2];
            double rollVelocity = gyro.getValues()[0];
            double pitchVelocity = gyro.getValues()[1];

            setIntermittentFrontalLeds((int) time);

            cameraRollMotor.setPosition(-0.115 * rollVelocity);
            cameraPitchMotor.setPosition(-0.1 * pitchVelocity);

            startRecognitionObjects();

            KeyboardShortcut movementByKeyboards = getKeyboardShortcut();

            double rollInput = kRollP * clamp(roll, -1.0, 1.0) + rollVelocity + movementByKeyboards.rollDisturbance();
            double pitchInput = kPitchP * clamp(pitch, -1.0, 1.0) + pitchVelocity + movementByKeyboards.pitchDisturbance();
            double yawInput = movementByKeyboards.yawDisturbance();
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

    public void displaySearchOptions() {
        Scanner scanner = new Scanner(System.in);
        System.out.println("---------------------------");
        System.out.println("Select a search option:");
        System.out.println("1. Manual drive");
        System.out.println("2. Drive by instructions");

        String input = scanner.nextLine();

        switch (input) {
            case "1":
                System.out.println("Manual drive selected");
                initControlByKeyboard();
                break;
            case "2":
                System.out.println("Drive by instructions selected");
                System.out.println("1. espiral");
                System.out.println("2. barrido");
                System.out.println("3. barrido con X uav");
                break;
            default:
                System.out.println("Invalid option, select 1 or 2");
                break;
        }

        scanner.close();
    }

    private KeyboardShortcut getKeyboardShortcut() {
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
        return new KeyboardShortcut(rollDisturbance, pitchDisturbance, yawDisturbance);
    }

    private record KeyboardShortcut(double rollDisturbance, double pitchDisturbance, double yawDisturbance) {
    }

    private void startRecognitionObjects() {
        int numberOfObjects = camera.getRecognitionNumberOfObjects();
        var objects = camera.getRecognitionObjects();

        for (int i = 0; i < numberOfObjects; i++) {
            int objectId = objects[i].getId();
            if (!recognizedObjectIds.contains(objectId)) {
                recognizedObjectIds.add(objectId);
                System.out.println("Model of object identified: " + objects[i].getModel());
                System.out.println("Id of object: " + objectId);
                System.out.println("Relative position of object: " + objects[i].getPosition()[0] + " " +
                        objects[i].getPosition()[1] + " " + objects[i].getPosition()[2]);
                System.out.println("Relative orientation of object: " + objects[i].getOrientation()[0] + " " +
                        objects[i].getOrientation()[1] + " " + objects[i].getOrientation()[2] + " " + objects[i].getOrientation()[3]);
                System.out.println("Size of object: " + objects[i].getSize()[0] + " " + objects[i].getSize()[1]);
                System.out.println("Position of the object on the camera image: " + objects[i].getPositionOnImage()[0] + " " +
                        objects[i].getPositionOnImage()[1]);
                System.out.println("Size of the object on the camera image: " + objects[i].getSizeOnImage()[0] + " " +
                        objects[i].getSizeOnImage()[1]);

                // Play sound alert
                // Speaker.playSound(speaker, speaker, "/Users/TFG/Documents/TFG/backend/dji-connection/src/main/resources/sounds/siren.wav", 1.0, 1.0, 0.0, true);

                // Save the image of the recognized object
                String filename = "object_" + objectId + ".jpg";
                camera.saveImage(filename, 100);
                System.out.println("Image saved as: " + filename);
            }
        }
    }

    private void initializeComponents() {
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

        // speaker = getSpeaker("speaker");
    }

    private void setIntermittentFrontalLeds(int time) {
        boolean ledState = time % 2 == 0;
        frontLeftLed.set(ledState ? 1 : 0);
        frontRightLed.set(ledState ? 0 : 1);
    }

    private static void printInstructions() {
        System.out.println("You can control the drone with the keyboard:");
        System.out.println("- 'up': move forward");
        System.out.println("- 'down': move backward");
        System.out.println("- 'right': turn right");
        System.out.println("- 'left': turn left");
        System.out.println("- 'shift + up': increase the target altitude");
        System.out.println("- 'shift + down': decrease the target altitude");
        System.out.println("- 'shift + right': strafe right");
        System.out.println("- 'shift + left': strafe left");
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
