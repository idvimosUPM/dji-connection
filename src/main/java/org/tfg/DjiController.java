package org.tfg;

import com.cyberbotics.webots.controller.*;
import com.cyberbotics.webots.controller.vehicle.Car;

public class DjiController extends Robot{

    private int timeStep = 500;
    private Camera camera = getCamera("camera");

    private LED frontLeftLed = getLED("front left led");
    private LED frontRightLed = getLED("front right led");

    private Motor motor1 = getMotor("front left propeller");
    private Motor motor2 = getMotor("front right propeller");
    private Motor motor3 = getMotor("rear left propeller");
    private Motor motor4 = getMotor("rear right propeller");

    void run() throws InterruptedException {

        while (step(timeStep) != -1) {

            frontRightLed.set(1);
            frontLeftLed.set(1);

            motor1.setVelocity(100.0);
            motor2.setVelocity(100.0);
            motor3.setVelocity(100.0);
            motor4.setVelocity(100.0);

        }
    }
    public static void main(String[] args) throws InterruptedException {
        new DjiController().run();
    }

}
