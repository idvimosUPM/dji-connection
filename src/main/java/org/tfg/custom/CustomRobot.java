package org.tfg.custom;

import com.cyberbotics.webots.controller.Accelerometer;
import com.cyberbotics.webots.controller.Altimeter;
import com.cyberbotics.webots.controller.Brake;
import com.cyberbotics.webots.controller.Camera;
import com.cyberbotics.webots.controller.Compass;
import com.cyberbotics.webots.controller.Connector;
import com.cyberbotics.webots.controller.Device;
import com.cyberbotics.webots.controller.Display;
import com.cyberbotics.webots.controller.DistanceSensor;
import com.cyberbotics.webots.controller.Emitter;
import com.cyberbotics.webots.controller.GPS;
import com.cyberbotics.webots.controller.Gyro;
import com.cyberbotics.webots.controller.InertialUnit;
import com.cyberbotics.webots.controller.LED;
import com.cyberbotics.webots.controller.Lidar;
import com.cyberbotics.webots.controller.LightSensor;
import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.Pen;
import com.cyberbotics.webots.controller.PositionSensor;
import com.cyberbotics.webots.controller.Radar;
import com.cyberbotics.webots.controller.RangeFinder;
import com.cyberbotics.webots.controller.Receiver;
import com.cyberbotics.webots.controller.Robot;
import com.cyberbotics.webots.controller.Skin;
import com.cyberbotics.webots.controller.Speaker;
import com.cyberbotics.webots.controller.TouchSensor;
import com.cyberbotics.webots.controller.VacuumGripper;
import lombok.Data;
import lombok.EqualsAndHashCode;
import lombok.NoArgsConstructor;

@EqualsAndHashCode(callSuper = true)
@Data
@NoArgsConstructor
public class CustomRobot extends Robot {

    // Step methods
    public int step(int var1) {
        return super.step(var1);
    }

    public int stepBegin(int var1) {
        return super.stepBegin(var1);
    }

    public int stepEnd() {
        return super.stepEnd();
    }

    // User input methods
    public int waitForUserInputEvent(int var1, int var2) {
        return super.waitForUserInputEvent(var1, var2);
    }

    // Information retrieval methods
    public String getName() {
        return super.getName();
    }

    public String getUrdf(String var1) {
        return super.getUrdf(var1);
    }

    public String getUrdf() {
        return super.getUrdf();
    }

    public double getTime() {
        return super.getTime();
    }

    public String getModel() {
        return super.getModel();
    }

    public String getCustomData() {
        return super.getCustomData();
    }

    public int getMode() {
        return super.getMode();
    }

    public boolean getSupervisor() {
        return super.getSupervisor();
    }

    public boolean getSynchronization() {
        return super.getSynchronization();
    }

    public String getProjectPath() {
        return super.getProjectPath();
    }

    public String getWorldPath() {
        return super.getWorldPath();
    }

    public double getBasicTimeStep() {
        return super.getBasicTimeStep();
    }

    public int getNumberOfDevices() {
        return super.getNumberOfDevices();
    }

    public Device getDevice(String var1) {
        return super.getDevice(var1);
    }

    public String wwiReceiveText() {
        return super.wwiReceiveText();
    }

    public String getData() {
        return super.getData();
    }

    // Device creation methods
    protected Accelerometer createAccelerometer(String var1) {
        return super.createAccelerometer(var1);
    }

    protected Altimeter createAltimeter(String var1) {
        return super.createAltimeter(var1);
    }

    protected Brake createBrake(String var1) {
        return super.createBrake(var1);
    }

    protected Camera createCamera(String var1) {
        return super.createCamera(var1);
    }

    protected Compass createCompass(String var1) {
        return super.createCompass(var1);
    }

    protected Connector createConnector(String var1) {
        return super.createConnector(var1);
    }

    protected Display createDisplay(String var1) {
        return super.createDisplay(var1);
    }

    protected DistanceSensor createDistanceSensor(String var1) {
        return super.createDistanceSensor(var1);
    }

    protected Emitter createEmitter(String var1) {
        return super.createEmitter(var1);
    }

    protected GPS createGPS(String var1) {
        return super.createGPS(var1);
    }

    protected Gyro createGyro(String var1) {
        return super.createGyro(var1);
    }

    protected InertialUnit createInertialUnit(String var1) {
        return super.createInertialUnit(var1);
    }

    protected LED createLED(String var1) {
        return super.createLED(var1);
    }

    protected Lidar createLidar(String var1) {
        return super.createLidar(var1);
    }

    protected LightSensor createLightSensor(String var1) {
        return super.createLightSensor(var1);
    }

    protected Motor createMotor(String var1) {
        return super.createMotor(var1);
    }

    protected Pen createPen(String var1) {
        return super.createPen(var1);
    }

    protected PositionSensor createPositionSensor(String var1) {
        return super.createPositionSensor(var1);
    }

    protected Radar createRadar(String var1) {
        return super.createRadar(var1);
    }

    protected RangeFinder createRangeFinder(String var1) {
        return super.createRangeFinder(var1);
    }

    protected Receiver createReceiver(String var1) {
        return super.createReceiver(var1);
    }

    protected Skin createSkin(String var1) {
        return super.createSkin(var1);
    }

    protected Speaker createSpeaker(String var1) {
        return super.createSpeaker(var1);
    }

    protected TouchSensor createTouchSensor(String var1) {
        return super.createTouchSensor(var1);
    }

    protected VacuumGripper createVacuumGripper(String var1) {
        return super.createVacuumGripper(var1);
    }

    // Battery sensor methods
    public void batterySensorEnable(int var1) {
        super.batterySensorEnable(var1);
    }

    public void batterySensorDisable() {
        super.batterySensorDisable();
    }

    public int batterySensorGetSamplingPeriod() {
        return super.batterySensorGetSamplingPeriod();
    }

    public double batterySensorGetValue() {
        return super.batterySensorGetValue();
    }

    // Data methods
    public void setCustomData(String var1) {
        super.setCustomData(var1);
    }

    public void setData(String var1) {
        super.setData(var1);
    }

    // WWI methods
    public void wwiSendText(String var1) {
        super.wwiSendText(var1);
    }

    // Protected methods
    protected CustomRobot(long var1, boolean var3) {
        super(var1, var3);
    }

    protected static long getCPtr(CustomRobot var0) {
        return Robot.getCPtr(var0);
    }

    protected static long swigRelease(CustomRobot var0) {
        return Robot.swigRelease(var0);
    }

    protected void finalize() {
        super.finalize();
    }

    public void delete() {
        super.delete();
    }
}