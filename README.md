# dji-connection
Code to add external Webots library into IntelliJ java project. In this case we will use Dji Mavic 2 PRO dron.

Java project version required is Java 17 with Gradle 8.8

To build this project it is mandatory to add the VM option to the project configuration:

-Djava.library.path=/Applications/Webots.app/Contents/lib/controller/java

This is the path to the Webots library that is needed to run the project.

This VM argument should be added in the main class to control the vehicle.