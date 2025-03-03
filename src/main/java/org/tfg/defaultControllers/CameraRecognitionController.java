package org.tfg.defaultControllers;

import com.cyberbotics.webots.controller.Camera;
import com.cyberbotics.webots.controller.CameraRecognitionObject;
import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.Robot;

public class CameraRecognitionController extends Robot {

    private static final double SPEED = 1.5;
    private static final int TIME_STEP = 64;

    public static void main(String[] args) {
        CameraRecognitionController controller = new CameraRecognitionController();
        controller.run();
    }

    public void run() {
        Camera camera = getCamera("camera");
        camera.enable(TIME_STEP);
        camera.recognitionEnable(TIME_STEP);

        Motor leftMotor = getMotor("left wheel motor");
        Motor rightMotor = getMotor("right wheel motor");
        leftMotor.setPosition(Double.POSITIVE_INFINITY);
        rightMotor.setPosition(Double.POSITIVE_INFINITY);

        leftMotor.setVelocity(-SPEED);
        rightMotor.setVelocity(SPEED);

        while (step(TIME_STEP) != -1) {
            int numberOfObjects = camera.getRecognitionNumberOfObjects();
            System.out.println("\nRecognized " + numberOfObjects + " objects.");

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
        }
    }
}