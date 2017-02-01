/* Copyright (c) 2015 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.ftcrobotcontroller.R;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.LightSensor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

/*
 *
 * This is an example LinearOpMode that shows how to use
 * a Modern Robotics Color Sensor.
 *
 * The op mode assumes that the color sensor
 * is configured with a name of "color sensor".
 *
 * You can use the X button on gamepad1 to toggle the LED on and off.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@TeleOp(name = "MR Color Beacon Push Test", group = "Test")
@Disabled
public class SensorMRColorBeaconPushTest extends LinearOpMode {

    VuforiaLocalizer vuforiaLocalizer;
    VuforiaLocalizer.Parameters parameters;
    VuforiaTrackables visionTargets;
    VuforiaTrackable target;
    VuforiaTrackableDefaultListener listener;

    OpenGLMatrix lastKnownLocation;
    OpenGLMatrix phoneLocation;

    public static final String VUFORIA_KEY = "AbeG4Kr/////AAAAGYlYzgmqaUCvl9MEIXe5q7VG+bqPbYy/jH9ZIRe+MDld1JqQEnOy1ljD1xH4lUXpV5DRpXq3iD0LcgRxjm+mTWaIxZ7jB2GroVg6Nn/HbRwnDNTWqC3UBjNAuGjAlUUbMg/CC69vQXKcZl11f97ly/RIj2leGI5MimrUsaTqIcNOQe6UUnkof1LFcpT+18Z7OhqWeRnJJDh9krwMceY1TNW7IwgB+vCrFp25jeQJF9cwclsnieT/NHhjdratCGWhCdU2FHe6mQiO+pjcT2X0suJPdtwomWCVqGDg6clj0A/yHUOrC5YHJ/RROMvAidn1Uo0a1OXL2nguV0jVA/rUeqIEYxTvS5OV0ePxeXlcrAhS"; // Insert your own key here


    ColorSensor colorSensor;    // Hardware Device Object
    HardwareBrainybot robot           = new HardwareBrainybot();
    private ElapsedTime runtime = new ElapsedTime();
    OpticalDistanceSensor odsSensor;  // Hardware Device Object
    OpticalDistanceSensor odsSensorRight;  // Hardware Device Object
    LightSensor lightSensor;
    LightSensor lightSensorBack;

    static final double SLIDE_SPEED = -0.5; // SLIDE_SPEED always Negative value
    static final double SLOWER_SLIDE_SPEED = -0.3; // SLIDE_SPEED always Negative value
    static final double SLOWEST_SLIDE_SPEED = -0.1; // SLIDE_SPEED always Negative value
    static final double FORWARD_SPEED = -0.6;
    static final double SLOWER_FORWARD_SPEED = -0.3;
    static final double SLOWERST_FORWARD_SPEED = -0.1;
    static final double ROTATE_SPEED = -0.1;

    static final double WHITE_THRESHOLD = 1.4;  // spans between 0.1 - 0.5 from dark to light

    //static final double WHEELS_X = 472.1;
    static final double WHEELS_X = 600.1;
    static final double WHEELS_Y = 480.1;
    static final double WHEELS_Z = 90.1;

    // Robot's X, Y, Z coordinate values
    double robotX = 0.0;
    double robotY = 0.0;
    double robotBearing = 0.0;
    double robotYToUse = 0.0;

    boolean buttonPushed = false;


  @Override
  public void runOpMode() throws InterruptedException {
      setupVuforia();
      sleep(3000);
      visionTargets.activate();

      // We don't know where the robot is, so set it to the origin
      // If we don't include this, it would be null, which would cause errors later on
      lastKnownLocation = createMatrix(0, 0, 0, 0, 0, 0);

    // hsvValues is an array that will hold the hue, saturation, and value information.
    float hsvValues[] = {0F,0F,0F};

    // values is a reference to the hsvValues array.
    final float values[] = hsvValues;

    // get a reference to the RelativeLayout so we can change the background
    // color of the Robot Controller app to match the hue detected by the RGB sensor.
    final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(R.id.RelativeLayout);

    // bPrevState and bCurrState represent the previous and current state of the button.
    boolean bPrevState = false;
    boolean bCurrState = false;

    // bLedOn represents the state of the LED.
    boolean bLedOn = false;

    // get a reference to our ColorSensor object.
    colorSensor = hardwareMap.colorSensor.get("color sensor");

    // get a reference to our Distance Sensor object.
    odsSensor = hardwareMap.opticalDistanceSensor.get("ods");
    odsSensorRight = hardwareMap.opticalDistanceSensor.get("ods right");

    lightSensor = hardwareMap.lightSensor.get("light sensor");
    lightSensorBack = hardwareMap.lightSensor.get("light sensor back");

    // Set the LED in the beginning
    colorSensor.enableLed(bLedOn);

    // turn on LED of light sensor.
    lightSensor.enableLed(true);
    lightSensorBack.enableLed(true);

    robot.init(hardwareMap);

    // Send telemetry message to signify robot waiting;
    telemetry.addData("Status", "Hello Brainy Bots. Ready to run!");    //
    telemetry.update();

    // After Init is pressed, this keeps updating the light values
    // GOOD, before the start, it caputures the values read.
    while (!(isStarted() || isStopRequested())) {
        // Get initial vision info
        // Ask the listener for the latest information on where the robot is
        OpenGLMatrix latestLocation = listener.getUpdatedRobotLocation();

        // The listener will sometimes return null, so we check for that to prevent errors
        if (latestLocation != null) {
            lastKnownLocation = latestLocation;
            // Then you can extract the positions and angles using the getTranslation and getOrientation methods.
            VectorF trans = lastKnownLocation.getTranslation();
            Orientation rot = Orientation.getOrientation(lastKnownLocation, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
            // Robot position is defined by the standard Matrix translation (x and y)
            robotX = trans.get(0);
            robotY = trans.get(1);

            // Robot bearing (in Cartesian system) position is defined by the standard Matrix z rotation
            robotBearing = rot.thirdAngle;
            if (robotBearing < 0) {
                robotBearing = 360 + robotBearing;
            }
        }
        else {
            telemetry.addData("Pos   ", "Unknown");
        }

        telemetry.addData("ODS Raw",    odsSensor.getRawLightDetected());
        telemetry.addData("ODS Right Raw",    odsSensorRight.getRawLightDetected());
        telemetry.addData("Light Raw Level", lightSensor.getRawLightDetected());
        telemetry.addData("Light Back Raw Level", lightSensor.getRawLightDetected());
        telemetry.addData("robotX",   robotX);
        telemetry.addData("robotY   ",   robotY);
        telemetry.addData("Bear  ", robotBearing);
        telemetry.addData("Pos   ", formatMatrix(lastKnownLocation));
        telemetry.addData("Tracking " + target.getName(), listener.isVisible());
        telemetry.update();
        idle();
    }

    // wait for the start button to be pressed.
    waitForStart();

      // mecanum wheel zig/zag testing.
      goForward(FORWARD_SPEED, 2.2);
      sleep(5000);
      // Approach from left of white line
      // Front wheels to right
      frontWheelsToRight(SLOWER_SLIDE_SPEED, lightSensor);
      sleep(1000);
      // Back wheels to right
      backWheelsToRight(SLOWER_SLIDE_SPEED, lightSensorBack);
      sleep(5000);

      /*
      // Approach from right of whilte line
      //Front wheels to left
      frontWheelsToLeft(SLOWER_SLIDE_SPEED, lightSensor);
      sleep(1000);
      // Back wheels to left
      backWheelsToLeft(SLOWER_SLIDE_SPEED, lightSensorBack);
      sleep(5000);
        */


      /*
      telemetry.addData("RobotX", robotX);
      telemetry.addData("RobotY", robotY);
      telemetry.addData("RobotZ", robotX);
      telemetry.addData("WHEELS_X",   WHEELS_X);
      telemetry.addData("WHEELS_Y",   WHEELS_Y);
      telemetry.addData("WHEELS_Z",   WHEELS_Z);
      telemetry.update();
      sleep(5000);
      */
      //telemetry.addData("Status",   "Testing NXT Light Sensor");
      //telemetry.update();
      //sleep(5000);


      //// TEST LIGHT SENSOR MOVE HERE!
      // if robotY < WHEELS_Y, then slideRight, while, stop
      // else slideLeft, while // STOP
      // May need failsafe
      // WORKS!!
      /*
      robot.armMotor.setPower(FORWARD_SPEED);
      robot.tableMotor.setPower(FORWARD_SPEED);
      robot.leftMotor.setPower(FORWARD_SPEED);
      robot.rightMotor.setPower(FORWARD_SPEED);
      while (opModeIsActive() && (lightSensor.getRawLightDetected() < WHITE_THRESHOLD)) {

          // Display the light level while we are looking for the line
          telemetry.addData("Light Level",  lightSensor.getRawLightDetected());
          telemetry.update();
          idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
      }
      goStop();
      */

      // telemetry.addData("Tracking " + target.getName(), listener.isVisible());
      //sleep(10000);

      /////////////////
      /// Includes to start from the beginning at the wall
      // Go forward
      goForward(FORWARD_SPEED, 1.1);
      sleep(1000);

      // Turn right
      turnRight(FORWARD_SPEED, 0.6);
      sleep(1000);

      // Go forward
      goForward(FORWARD_SPEED, 2.0);

      sleep(5000);

      // NOTE if THIS FAILS, EVERYTHING FAILS!!
      turnRight(FORWARD_SPEED, 0.3);

    // This doesnt appear to work, try to look for target. Too quick the image.
     /* while (opModeIsActive() && (!listener.isVisible())) {
          // Ask the listener for the latest information on where the robot is
          OpenGLMatrix latestLocation = listener.getUpdatedRobotLocation();

          // The listener will sometimes return null, so we check for that to prevent errors
          if (latestLocation != null) {
              lastKnownLocation = latestLocation;
              // Then you can extract the positions and angles using the getTranslation and getOrientation methods.
              VectorF trans = lastKnownLocation.getTranslation();
              Orientation rot = Orientation.getOrientation(lastKnownLocation, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
              // Robot position is defined by the standard Matrix translation (x and y)
              robotX = trans.get(0);
              robotY = trans.get(1);

              // Robot bearing (in Cartesian system) position is defined by the standard Matrix z rotation
              robotBearing = rot.thirdAngle;
              if (robotBearing < 0) {
                  robotBearing = 360 + robotBearing;
              }
          }
          else {
              telemetry.addData("Pos   ", "Unknown");
          }
          robot.armMotor.setPower(SLOWERST_FORWARD_SPEED);
          robot.tableMotor.setPower(SLOWERST_FORWARD_SPEED);
          robot.leftMotor.setPower(-SLOWER_FORWARD_SPEED);
          robot.rightMotor.setPower(-SLOWER_FORWARD_SPEED);
          runtime.reset();
          while (opModeIsActive() && (runtime.seconds() < 0.2)) {
              telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
              telemetry.update();
              idle();
          }
          goStop();
          sleep(5000); //=> This might allow the images to be found?
          telemetry.addData("Status", "Looking for target...");
          telemetry.update();
          idle();
      }
      */
      // Stop
      //goStop();

      // Go backward
      //goForward(-SLOWER_FORWARD_SPEED, 0.5);

      // Turn right
      //turnRight(SLOWER_FORWARD_SPEED, 0.6);

      /////////
      ////////
      /////////

      // Get one time vision info before rotating
      // Get initial vision info
      // Ask the listener for the latest information on where the robot is


      // First, rotate.
      telemetry.addData("Status", "Check these before rotating");
      telemetry.addData("RobotX", robotX);
      telemetry.addData("RobotY", robotY);
      telemetry.addData("RobotZ", robotBearing);
      telemetry.addData("WHEELS_X",   WHEELS_X);
      telemetry.addData("WHEELS_Y",   WHEELS_Y);
      telemetry.addData("WHEELS_Z",   WHEELS_Z);
      telemetry.update();
      sleep(5000);
      telemetry.addData("Status", "rotating to Left!");
      telemetry.update();
      sleep(5000);

      // BUG - CURRENTLY NOT WORKING RIGHT. TURNS TOO MUCH to LEFT!
      // If robotZ > WHEELS_Z, then rotateLeft
      // else rotateRight
      //////////////
      // START Z MOVE
      /////////////
      // CASE: rotateLeft to image
      if (robotBearing < WHEELS_Z) {
          robot.armMotor.setPower(-ROTATE_SPEED);
          robot.tableMotor.setPower(-ROTATE_SPEED);
          robot.leftMotor.setPower(ROTATE_SPEED);
          robot.rightMotor.setPower(ROTATE_SPEED);
          while (opModeIsActive() && (robotBearing < WHEELS_Z)) { // GOOD to stop close to recognize color
              // Ask the listener for the latest information on where the robot is
              OpenGLMatrix latestLocation = listener.getUpdatedRobotLocation();

              if (latestLocation != null) {
                  lastKnownLocation = latestLocation;
                  // Then you can extract the positions and angles using the getTranslation and getOrientation methods.
                  VectorF trans = lastKnownLocation.getTranslation();
                  Orientation rot = Orientation.getOrientation(lastKnownLocation, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
                  // Robot position is defined by the standard Matrix translation (x and y)
                  robotX = trans.get(0);
                  robotY = trans.get(1);

                  // Robot bearing (in Cartesian system) position is defined by the standard Matrix z rotation
                  robotBearing = rot.thirdAngle;
                  if (robotBearing < 0) {
                      robotBearing = 360 + robotBearing;
                  }
              }
              else {
                  telemetry.addData("Pos   ", "Unknown");
              }

              telemetry.addData("RobotX", robotX);
              telemetry.addData("RobotY ",   robotY);
              telemetry.addData("Bear  ", robotBearing);
              telemetry.addData("Pos   ", formatMatrix(lastKnownLocation));
              telemetry.addData("Status", "In robotBearing while loop");
              telemetry.update();
              robotYToUse = robotY;
              idle();
              //sleep(10000);

          }
      }
      else { // rotateRight
          robot.armMotor.setPower(ROTATE_SPEED);
          robot.tableMotor.setPower(ROTATE_SPEED);
          robot.leftMotor.setPower(-ROTATE_SPEED);
          robot.rightMotor.setPower(-ROTATE_SPEED);
          while (opModeIsActive() && (robotBearing < WHEELS_Z)) { // GOOD to stop close to recognize color
              // Ask the listener for the latest information on where the robot is
              OpenGLMatrix latestLocation = listener.getUpdatedRobotLocation();

              if (latestLocation != null) {
                  lastKnownLocation = latestLocation;
                  // Then you can extract the positions and angles using the getTranslation and getOrientation methods.
                  VectorF trans = lastKnownLocation.getTranslation();
                  Orientation rot = Orientation.getOrientation(lastKnownLocation, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
                  // Robot position is defined by the standard Matrix translation (x and y)
                  robotX = trans.get(0);
                  robotY = trans.get(1);

                  // Robot bearing (in Cartesian system) position is defined by the standard Matrix z rotation
                  robotBearing = rot.thirdAngle;
                  if (robotBearing < 0) {
                      robotBearing = 360 + robotBearing;
                  }
              }
              else {
                  telemetry.addData("Pos   ", "Unknown");
              }

              telemetry.addData("RobotX", robotX);
              telemetry.addData("RobotY ",   robotY);
              telemetry.addData("Bear  ", robotBearing);
              telemetry.addData("Pos   ", formatMatrix(lastKnownLocation));
              telemetry.addData("Status", "In robotBearing while loop");
              telemetry.update();
              robotYToUse = robotY;
              idle();
              //sleep(10000);

          }
      }


      goStop();
      //////////////
      // END Z MOVE
      /////////////
      telemetry.addData("Status", "Check robotBear");
      telemetry.addData("Bear  ", robotBearing);
      telemetry.addData("RobotX", robotX);
      telemetry.addData("RobotY ",   robotY);
      telemetry.addData("Bear  ", robotBearing);
      telemetry.addData("Pos   ", formatMatrix(lastKnownLocation));
      telemetry.addData("Status", "In robotBearing while loop");
      telemetry.update();
      sleep(10000);

      // Second, goForward closer to the beacon.
      //////////////
      // START X MOVE
      /////////////
      // X MOVE START: Vision target acquired, move then to the preknown location before it loses frame
      robot.armMotor.setPower(SLOWERST_FORWARD_SPEED);
      robot.tableMotor.setPower(SLOWERST_FORWARD_SPEED);
      robot.leftMotor.setPower(SLOWERST_FORWARD_SPEED);
      robot.rightMotor.setPower(SLOWERST_FORWARD_SPEED);
      while (opModeIsActive() && (robotX > WHEELS_X)) { // GOOD to stop close to recognize color
          // Ask the listener for the latest information on where the robot is
          OpenGLMatrix latestLocation = listener.getUpdatedRobotLocation();

          if (latestLocation != null) {
              lastKnownLocation = latestLocation;
              // Then you can extract the positions and angles using the getTranslation and getOrientation methods.
              VectorF trans = lastKnownLocation.getTranslation();
              Orientation rot = Orientation.getOrientation(lastKnownLocation, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
              // Robot position is defined by the standard Matrix translation (x and y)
              robotX = trans.get(0);
              robotY = trans.get(1);

              // Robot bearing (in Cartesian system) position is defined by the standard Matrix z rotation
              robotBearing = rot.thirdAngle;
              if (robotBearing < 0) {
                  robotBearing = 360 + robotBearing;
              }
          }
          else {
              telemetry.addData("Pos   ", "Unknown");
          }

          telemetry.addData("RobotX", robotX);
          telemetry.addData("RobotY ",   robotY);
          telemetry.addData("Bear  ", robotBearing);
          telemetry.addData("Pos   ", formatMatrix(lastKnownLocation));
          telemetry.addData("Status", "In robotX while loop");
          telemetry.update();
          idle();
          //sleep(10000);

      }

      // GOOD, without these stop, the robot does not stop from above.
      goStop();
      //////////////
      // END X MOVE
      /////////////
      telemetry.addData("RobotX", robotX);
      telemetry.addData("RobotY ",   robotY);
      telemetry.addData("Bear  ", robotBearing);
      telemetry.addData("Pos   ", formatMatrix(lastKnownLocation));
      telemetry.addData("Status", "In robotBearing while loop");
      telemetry.update();
      sleep(5000);


      telemetry.addData("Status", "Sliding to Left or right!");
      telemetry.update();
      sleep(5000);
      // Third, then do the line stop.


      //////////////
      // START Y MOVE
      /////////////
      // GOOD!
      // CASE: slideLeft to image
      if (robotY > WHEELS_Y) {
          robot.armMotor.setPower(-SLIDE_SPEED);
          robot.tableMotor.setPower(SLIDE_SPEED);
          robot.leftMotor.setPower(-SLIDE_SPEED);
          robot.rightMotor.setPower(SLIDE_SPEED);
          while (opModeIsActive() && (robotY > WHEELS_Y)) { // GOOD to stop close to recognize color
              // Ask the listener for the latest information on where the robot is
              OpenGLMatrix latestLocation = listener.getUpdatedRobotLocation();

              if (latestLocation != null) {
                  lastKnownLocation = latestLocation;
                  // Then you can extract the positions and angles using the getTranslation and getOrientation methods.
                  VectorF trans = lastKnownLocation.getTranslation();
                  Orientation rot = Orientation.getOrientation(lastKnownLocation, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
                  // Robot position is defined by the standard Matrix translation (x and y)
                  robotX = trans.get(0);
                  robotY = trans.get(1);

                  // Robot bearing (in Cartesian system) position is defined by the standard Matrix z rotation
                  robotBearing = rot.thirdAngle;
                  if (robotBearing < 0) {
                      robotBearing = 360 + robotBearing;
                  }
              }
              else {
                  telemetry.addData("Pos   ", "Unknown");
              }

              telemetry.addData("RobotX", robotX);
              telemetry.addData("RobotY ",   robotY);
              telemetry.addData("Bear  ", robotBearing);
              telemetry.addData("Pos   ", formatMatrix(lastKnownLocation));
              telemetry.addData("Status", "In robotY while loop");
              telemetry.update();
              idle();
              //sleep(10000);

          }

          // STOP to recognize color.
          // GOOD, without these stop, the robot does not stop from above.
          goStop();
      }
      else { // Must need to slide right, NEED to see if it can zig zag
          robot.armMotor.setPower(SLIDE_SPEED);
          robot.tableMotor.setPower(-SLIDE_SPEED);
          robot.leftMotor.setPower(SLIDE_SPEED);
          robot.rightMotor.setPower(-SLIDE_SPEED);

          while (opModeIsActive() && (lightSensor.getRawLightDetected() < WHITE_THRESHOLD)) {

              // Display the light level while we are looking for the line
              telemetry.addData("Light Level",  lightSensor.getRawLightDetected());
              telemetry.addData("Light Back Level",  lightSensorBack.getRawLightDetected());
              telemetry.update();
              idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
          }
          /*
          while (opModeIsActive() && (robotY < WHEELS_Y)) { // GOOD to stop close to recognize color
              // Ask the listener for the latest information on where the robot is
              OpenGLMatrix latestLocation = listener.getUpdatedRobotLocation();

              if (latestLocation != null) {
                  lastKnownLocation = latestLocation;
                  // Then you can extract the positions and angles using the getTranslation and getOrientation methods.
                  VectorF trans = lastKnownLocation.getTranslation();
                  Orientation rot = Orientation.getOrientation(lastKnownLocation, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
                  // Robot position is defined by the standard Matrix translation (x and y)
                  robotX = trans.get(0);
                  robotY = trans.get(1);

                  // Robot bearing (in Cartesian system) position is defined by the standard Matrix z rotation
                  robotBearing = rot.thirdAngle;
                  if (robotBearing < 0) {
                      robotBearing = 360 + robotBearing;
                  }
              }
              else {
                  telemetry.addData("Pos   ", "Unknown");
              }

              telemetry.addData("RobotX", robotX);
              telemetry.addData("RobotY ",   robotY);
              telemetry.addData("Bear  ", robotBearing);
              telemetry.addData("Pos   ", formatMatrix(lastKnownLocation));
              telemetry.addData("Status", "In robotY while loop");
              telemetry.update();
              idle();
              //sleep(10000);

          }
            */
          // STOP to recognize color.
          // GOOD, without these stop, the robot does not stop from above.
          goStop();

          // Then, back wheels to zig zag to line up.
          /*
          //robot.armMotor.setPower(SLIDE_SPEED);
          robot.tableMotor.setPower(-SLIDE_SPEED);
          //robot.leftMotor.setPower(SLIDE_SPEED);
          robot.rightMotor.setPower(-SLIDE_SPEED);

          while (opModeIsActive() && (lightSensorBack.getRawLightDetected() < WHITE_THRESHOLD)) {

              // Display the light level while we are looking for the line
              telemetry.addData("Light Level",  lightSensor.getRawLightDetected());
              telemetry.addData("Light Back Level",  lightSensorBack.getRawLightDetected());
              telemetry.update();
              idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
          }
          goStop();
          */
      }

      //////////////
      // END Y MOVE
      /////////////

      /*
      telemetry.addData("Status", "rotating to Left!");
      telemetry.update();
      sleep(5000);

      //////////////
      // START Z MOVE
      /////////////
      // CASE: rotateLeft to image
      robot.armMotor.setPower(-ROTATE_SPEED);
      robot.tableMotor.setPower(-ROTATE_SPEED);
      robot.leftMotor.setPower(ROTATE_SPEED);
      robot.rightMotor.setPower(ROTATE_SPEED);
      while (opModeIsActive() && (robotBearing < WHEELS_Z)) { // GOOD to stop close to recognize color
          // Ask the listener for the latest information on where the robot is
          OpenGLMatrix latestLocation = listener.getUpdatedRobotLocation();

          if (latestLocation != null) {
              lastKnownLocation = latestLocation;
              // Then you can extract the positions and angles using the getTranslation and getOrientation methods.
              VectorF trans = lastKnownLocation.getTranslation();
              Orientation rot = Orientation.getOrientation(lastKnownLocation, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
              // Robot position is defined by the standard Matrix translation (x and y)
              robotX = trans.get(0);
              robotY = trans.get(1);

              // Robot bearing (in Cartesian system) position is defined by the standard Matrix z rotation
              robotBearing = rot.thirdAngle;
              if (robotBearing < 0) {
                  robotBearing = 360 + robotBearing;
              }
          }
          else {
              telemetry.addData("Pos   ", "Unknown");
          }

          telemetry.addData("RobotX", robotX);
          telemetry.addData("RobotY ",   robotY);
          telemetry.addData("Bear  ", robotBearing);
          telemetry.addData("Pos   ", formatMatrix(lastKnownLocation));
          telemetry.addData("Status", "In robotBearing while loop");
          telemetry.update();
          idle();
          //sleep(10000);

      }

      goStop();
      //////////////
      // END Z MOVE
      /////////////
        */


      // Change to Distance Sensor
      telemetry.addData("Status", "Switching to Distance Sensor!");
      telemetry.update();
      sleep(5000);


    // START: First staright move close to the button where it can recognize color
      robot.armMotor.setPower(SLOWERST_FORWARD_SPEED);
      robot.tableMotor.setPower(SLOWERST_FORWARD_SPEED);
      robot.leftMotor.setPower(SLOWERST_FORWARD_SPEED);
      robot.rightMotor.setPower(SLOWERST_FORWARD_SPEED);

      while (opModeIsActive() && (odsSensor.getRawLightDetected() < 0.15)) { // GOOD to stop close to recognize color
          //while (odsSensor.getRawLightDetected() < 0.1) {
          telemetry.addData("ODS Raw",    odsSensor.getRawLightDetected());
          telemetry.addData("ODS Right Raw",    odsSensorRight.getRawLightDetected());
          telemetry.addData("Status", "In first while loop");
          telemetry.update();
          //sleep(10000);
          idle();

      }

      // STOP to recognize color.
      // GOOD, without these stop, the robot does not stop from above.
      robot.armMotor.setPower(0.0);
      robot.tableMotor.setPower(0.0);
      robot.leftMotor.setPower(0.0);
      robot.rightMotor.setPower(0.0);
      telemetry.addData("Status", "FIRST stop");
      telemetry.addData("Red  ", colorSensor.red());
      telemetry.addData("Blue ", colorSensor.blue());
      telemetry.update();
      sleep(5000);
      // END: First staright move close to the button where it can recognize color

      // START: Recognize RED Color
      // Change to BLUE recognition
      //else if ((colorSensor.blue() >= 1) && (hsvValues[0] > 100) && (colorSensor.red() == 0)
      //&& (buttonPushed == false)
      if ((colorSensor.red() >= 1) && (colorSensor.blue() == 0) &&
              (buttonPushed == false)) {
          telemetry.addData("Status", "RED DETECTED");
          telemetry.update();
          sleep(5000);
          // START: Bump the button!!
          robot.armMotor.setPower(SLOWERST_FORWARD_SPEED);
          robot.tableMotor.setPower(SLOWERST_FORWARD_SPEED);
          robot.leftMotor.setPower(SLOWERST_FORWARD_SPEED);
          robot.rightMotor.setPower(SLOWERST_FORWARD_SPEED);

          // 0.40 is about hitting the button on beacon, if the distance sensor is right hand
          // side beacon detection, then this 0.40 is too much, about 0.20 would work.
          while (opModeIsActive() && (odsSensor.getRawLightDetected() < 0.40)) { // GOOD to stop close to recognize color
              //while (odsSensor.getRawLightDetected() < 0.1) {
              telemetry.addData("ODS Raw",    odsSensor.getRawLightDetected());
              //telemetry.addData("Status", "In first while loop");
              telemetry.update();
              //sleep(10000);

          }

          // STOP to recognize color.
          // GOOD, without these stop, the robot does not stop from above.
          robot.armMotor.setPower(0.0);
          robot.tableMotor.setPower(0.0);
          robot.leftMotor.setPower(0.0);
          robot.rightMotor.setPower(0.0);
          // END: Bump the button!!
      }
      else { //Must be BLUE Color. Slide left and bump the button with right bumper.
          telemetry.addData("Status", "RED NOT DETECTED, sliding to RIGHT!");
          telemetry.update();
          sleep(5000);
          slideLeft(SLIDE_SPEED, 0.5);
          sleep(1000);
          // Then bump button
          //goForward(SLOWERST_FORWARD_SPEED, 0.3);
          // START: Bump the button!!
          robot.armMotor.setPower(SLOWERST_FORWARD_SPEED);
          robot.tableMotor.setPower(SLOWERST_FORWARD_SPEED);
          robot.leftMotor.setPower(SLOWERST_FORWARD_SPEED);
          robot.rightMotor.setPower(SLOWERST_FORWARD_SPEED);

          // 0.40 is about hitting the button on beacon, if the distance sensor is right hand
          // side beacon detection, then this 0.40 is too much, about 0.20 would work.
          while (opModeIsActive() && (odsSensorRight.getRawLightDetected() < 0.40)) { // GOOD to stop close to recognize color
              //while (odsSensor.getRawLightDetected() < 0.1) {
              telemetry.addData("ODS Right Raw",    odsSensorRight.getRawLightDetected());
              //telemetry.addData("Status", "In first while loop");
              telemetry.update();
              //sleep(10000);

          }

          // STOP to recognize color.
          // GOOD, without these stop, the robot does not stop from above.
          robot.armMotor.setPower(0.0);
          robot.tableMotor.setPower(0.0);
          robot.leftMotor.setPower(0.0);
          robot.rightMotor.setPower(0.0);
          // END: Bump the button!!

      }
      // END: Recognize Color


      // Go back a bit after pressing the button
      telemetry.addData("Status", "Backing up");
      telemetry.update();
      sleep(3000);
      goForward(-FORWARD_SPEED, 0.4);

      // rotateLeft so front faces next white line on the next image
      turnLeft(FORWARD_SPEED, 1.0);
      sleep(1000);

      // Go and stop at the next white line on next image
      telemetry.addData("Status", "Going to detect next white line!!");
      telemetry.update();
      sleep(3000);
      goForwardTillLine(FORWARD_SPEED, lightSensor);
      //slideLeft(SLIDE_SPEED, 3.5);

      // rotateRight so it faces the beacon
      // After turn, the left color sensor should be at the left button.
      turnRight(FORWARD_SPEED, 1.0);

      //
      // Then, recognize the color and bump like before.
      // CODE NEEDED HERE!




/*
      robot.armMotor.setPower(FORWARD_SPEED);
      robot.tableMotor.setPower(FORWARD_SPEED);
      robot.leftMotor.setPower(FORWARD_SPEED);
      robot.rightMotor.setPower(FORWARD_SPEED);


      while (opModeIsActive() && (odsSensor.getRawLightDetected() < 0.2)) { // GOOD to stop close to recognize color
          //while (odsSensor.getRawLightDetected() < 0.1) {
          telemetry.addData("ODS Raw",    odsSensor.getRawLightDetected());

          telemetry.addData("Status", "In 2nd while loop");
          telemetry.update();
          //sleep(10000);

      }

      robot.armMotor.setPower(0.0);
      robot.tableMotor.setPower(0.0);
      robot.leftMotor.setPower(0.0);
      robot.rightMotor.setPower(0.0);
      telemetry.addData("Status", "2nd stop");
      sleep(5000);
*/

      /* BAD CODE - ROBOT DOES NOT STOP AT ALL, NO REFRESH OF SCREEN!!
      while (odsSensor.getRawLightDetected() < 0.24) {
          robot.armMotor.setPower(FORWARD_SPEED);
          robot.tableMotor.setPower(FORWARD_SPEED);
          robot.leftMotor.setPower(FORWARD_SPEED);
          robot.rightMotor.setPower(FORWARD_SPEED);

      }

      // STOP to recognize color.
      //robot.armMotor.setPower(0.0);
      //robot.tableMotor.setPower(0.0);
      //robot.leftMotor.setPower(0.0);
      //robot.rightMotor.setPower(0.0);
      telemetry.addData("Status", "Before 10 sec sleep");
      //telemetry.update();
      sleep(10000);
      telemetry.addData("Status", "after 10 sec sleep");
      //telemetry.update();
      */

      ////////////////////////////////////////
      // HERE STARTS THE ROBOT MOVE CODE!!
      // Go forward (until the distance is close)

      /*
      while (odsSensor.getLightDetected() < 0.04) {
          robot.armMotor.setPower(FORWARD_SPEED);
          robot.tableMotor.setPower(FORWARD_SPEED);
          robot.leftMotor.setPower(FORWARD_SPEED);
          robot.rightMotor.setPower(FORWARD_SPEED);
      }
      */
      ;
      /*
      // STOP to recognize color.
      runtime.reset();
      while (opModeIsActive() && (odsSensor.getLightDetected() > 0.04)) {
          telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
          telemetry.addData("ODS Raw",    odsSensor.getRawLightDetected());
          telemetry.addData("ODS Normal", odsSensor.getLightDetected());

          robot.armMotor.setPower(0.0);
          robot.tableMotor.setPower(0.0);
          robot.leftMotor.setPower(0.0);
          robot.rightMotor.setPower(0.0);

          telemetry.update();
          idle();
      }
      */
      /*
      if (odsSensor.getLightDetected() > 0.04) {
          robot.armMotor.setPower(0.0);
          robot.tableMotor.setPower(0.0);
          robot.leftMotor.setPower(0.0);
          robot.rightMotor.setPower(0.0);
      }
        */
    // while the op mode is active, loop and read the RGB data.
    // Note we use opModeIsActive() as our loop condition because it is an interruptible method.
    while (opModeIsActive()) {

      // check the status of the x button on either gamepad.
      //bCurrState = gamepad1.x;

      // check for button state transitions.
      //if ((bCurrState == true) && (bCurrState != bPrevState))  {

        // button is transitioning to a pressed state. So Toggle LED
        //bLedOn = !bLedOn;
        //colorSensor.enableLed(bLedOn);
      //}

      // update previous state variable.
      bPrevState = bCurrState;

      // convert the RGB values to HSV values.
      Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);

      // send the info back to driver station using telemetry function.
      telemetry.addData("LED", bLedOn ? "On" : "Off");
      telemetry.addData("Clear", colorSensor.alpha());
      telemetry.addData("Red  ", colorSensor.red());
      telemetry.addData("Green", colorSensor.green());
      telemetry.addData("Blue ", colorSensor.blue());
      telemetry.addData("Hue", hsvValues[0]);

      // change the background color to match the color detected by the RGB sensor.
      // pass a reference to the hue, saturation, and value array as an argument
      // to the HSVToColor method.
      relativeLayout.post(new Runnable() {
        public void run() {
          relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
        }
      });

      // send the info back to driver station using telemetry function.
      telemetry.addData("ODS Raw",    odsSensor.getRawLightDetected());
      telemetry.addData("ODS Normal", odsSensor.getLightDetected());
        telemetry.addData("ODS Right Raw",    odsSensorRight.getRawLightDetected());
        telemetry.addData("ODS Right Normal", odsSensorRight.getLightDetected());
      telemetry.addData("Button Pushed?", buttonPushed);
      telemetry.update(); // THIS CODE GOOD, so all values are on screen

      ////////////////////////////////////////
      // HERE STARTS THE ROBOT MOVE CODE!!
      // Go forward (until the distance is close)

        // THIS CODE STOPS, but it doesn't update screen until object goes away, then get close.
        // Then screen updates.

/*
       robot.armMotor.setPower(FORWARD_SPEED);
       robot.tableMotor.setPower(FORWARD_SPEED);
       robot.leftMotor.setPower(FORWARD_SPEED);
       robot.rightMotor.setPower(FORWARD_SPEED);


        while (opModeIsActive() && (odsSensor.getRawLightDetected() < 0.24)) { // GOOD to stop close to recognize color
        //while (odsSensor.getRawLightDetected() < 0.1) {
            telemetry.addData("ODS Raw",    odsSensor.getRawLightDetected());

            telemetry.addData("Status", "In first while loop");
            telemetry.update();
                //sleep(10000);

            }


        // STOP to recognize color.
        // GOOD, without these stop, the robot does not stop from above.
        robot.armMotor.setPower(0.0);
        robot.tableMotor.setPower(0.0);
        robot.leftMotor.setPower(0.0);
        robot.rightMotor.setPower(0.0);
        telemetry.addData("Status", "FIRST stop");
*/
        /*
        // 2nd step to get closer
        while ((odsSensor.getRawLightDetected() >= 0.1) && (odsSensor.getRawLightDetected() < 0.5)) {
            robot.armMotor.setPower(FORWARD_SPEED);
            robot.tableMotor.setPower(FORWARD_SPEED);
            robot.leftMotor.setPower(FORWARD_SPEED);
            robot.rightMotor.setPower(FORWARD_SPEED);
            telemetry.addData("Status", "In 2nd while loop");
            sleep(10000);


        }


        robot.armMotor.setPower(0.0);
        robot.tableMotor.setPower(0.0);
        robot.leftMotor.setPower(0.0);
        robot.rightMotor.setPower(0.0);
        telemetry.addData("Status", "2nd STOP");
        */



        /*
        // GOOD CODE, detects RED!
        if ((colorSensor.red() >= 3) && (colorSensor.blue() == 0) && (buttonPushed == false)) {
            telemetry.addData("Status", "RED DETECTED");
            sleep(5000);
        }
        else
            telemetry.addData("Status", "RED NOT DETECTED");
        */
        //telemetry.addData("Status", "Before 10 sec sleep");
        //telemetry.update();
        //sleep(10000);
        //telemetry.addData("Status", "after 10 sec sleep");
        //telemetry.update();

        /*
        if ((colorSensor.red() >= 3) && (colorSensor.blue() == 0) && (buttonPushed == false)) {
            // RED detected, go forward a bit 0.1 sec to press the button
            //while ((odsSensor.getRawLightDetected() >= 0.24) && (odsSensor.getRawLightDetected() < 0.80)) {
            telemetry.addData("Status", "In if ");

            while (odsSensor.getRawLightDetected() < 0.80) {
                robot.armMotor.setPower(FORWARD_SPEED);
                robot.tableMotor.setPower(FORWARD_SPEED);
                robot.leftMotor.setPower(FORWARD_SPEED);
                robot.rightMotor.setPower(FORWARD_SPEED);
                telemetry.addData("Status", "In if while loop");
                //telemetry.update();

            }

            // STOP to recognize color.
            robot.armMotor.setPower(0.0);
            robot.tableMotor.setPower(0.0);
            robot.leftMotor.setPower(0.0);
            robot.rightMotor.setPower(0.0);
            /*
            robot.armMotor.setPower(FORWARD_SPEED);
            robot.tableMotor.setPower(FORWARD_SPEED);
            robot.leftMotor.setPower(FORWARD_SPEED);
            robot.rightMotor.setPower(FORWARD_SPEED);

        if (odsSensor.getLightDetected() > 0.04) {
          robot.armMotor.setPower(0.0);
          robot.tableMotor.setPower(0.0);
          robot.leftMotor.setPower(0.0);
          robot.rightMotor.setPower(0.0);

        }
            buttonPushed = true;
            //telemetry.update();
            //sleep(20000);

      }
        telemetry.addData("Status", "OUT if while loop");
        */
        /*
      else if ((colorSensor.blue() >= 3) && (hsvValues[0] > 100) && (colorSensor.red() == 0)
              && (buttonPushed == false)) {
        // BLUE detected, slide and push the button
        robot.armMotor.setPower(0.5);
        robot.tableMotor.setPower(-0.5);
        robot.leftMotor.setPower(0.5);
        robot.rightMotor.setPower(-0.5);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.6)) {
          telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
          telemetry.update();
          idle();
        }

        // Stop
        robot.armMotor.setPower(0);
        robot.tableMotor.setPower(0);
        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);

        // Then bump the button.
        robot.armMotor.setPower(FORWARD_SPEED);
        robot.tableMotor.setPower(FORWARD_SPEED);
        robot.leftMotor.setPower(FORWARD_SPEED);
        robot.rightMotor.setPower(FORWARD_SPEED);

        while (opModeIsActive() && (runtime.seconds() < 0.5)) {
          telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
          telemetry.update();
          idle();
        }

        if (odsSensor.getRawLightDetected() > 0.04) {
          robot.armMotor.setPower(0.0);
          robot.tableMotor.setPower(0.0);
          robot.leftMotor.setPower(0.0);
          robot.rightMotor.setPower(0.0);
        }

        buttonPushed = true;
      }
        */
      /*else {
        // Stop
        robot.armMotor.setPower(0);
        robot.tableMotor.setPower(0);
        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);
      }*/

      /*
      // Go backward after pressing the button
      robot.armMotor.setPower(-FORWARD_SPEED);
      robot.tableMotor.setPower(-FORWARD_SPEED);
      robot.leftMotor.setPower(-FORWARD_SPEED);
      robot.rightMotor.setPower(-FORWARD_SPEED);
      while (opModeIsActive() && (runtime.seconds() < 1.0)) {
        telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
        telemetry.update();
        idle();
      }
      // Stop
      robot.armMotor.setPower(0);
      robot.tableMotor.setPower(0);
      robot.leftMotor.setPower(0);
      robot.rightMotor.setPower(0);
        */


      telemetry.update();
      idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
    }
  }

    public void goStop()
    {
        // This function stops all the moving motors.
        robot.armMotor.setPower(0);
        robot.tableMotor.setPower(0);
        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);
    }

    public void goForward(double speed, double duration) throws InterruptedException
    {
        // This function allows robot to go forward with speed, time duration argument.

        robot.armMotor.setPower(speed);
        robot.tableMotor.setPower(speed);
        robot.leftMotor.setPower(speed);
        robot.rightMotor.setPower(speed);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < duration)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
            idle();
        }

        // Stop
        goStop();
    }

    public void goForwardTillLine(double speed, LightSensor sensor) throws InterruptedException
    {
        // This function allows robot to go forward with speed, time duration argument.

        robot.armMotor.setPower(speed);
        robot.tableMotor.setPower(speed);
        robot.leftMotor.setPower(speed);
        robot.rightMotor.setPower(speed);
        while (opModeIsActive() && (sensor.getRawLightDetected() < WHITE_THRESHOLD)) {

            // Display the light level while we are looking for the line
            telemetry.addData("Light Level",  sensor.getRawLightDetected());
            telemetry.update();
            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }

        // Stop
        goStop();
    }

    public void turnRight(double speed, double duration) throws InterruptedException
    {
        // This function allows robot to turn right with speed, time duration argument.
        robot.armMotor.setPower(speed);
        robot.tableMotor.setPower(speed);
        robot.leftMotor.setPower(-speed);
        robot.rightMotor.setPower(-speed);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < duration)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
            idle();
        }

        // Stop
        goStop();
    }

    public void turnLeft(double speed, double duration) throws InterruptedException
    {
        // This function allows robot to turn right with speed, time duration argument.
        robot.armMotor.setPower(-speed);
        robot.tableMotor.setPower(-speed);
        robot.leftMotor.setPower(speed);
        robot.rightMotor.setPower(speed);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < duration)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
            idle();
        }

        // Stop
        goStop();
    }

    public void slideRight(double speed, double duration) throws InterruptedException
    {
        // Slide right
        //robot.armMotor.setPower(0.5);
        //robot.tableMotor.setPower(-0.5);
        //robot.leftMotor.setPower(0.5);
        //robot.rightMotor.setPower(-0.5);
        robot.armMotor.setPower(speed);
        robot.tableMotor.setPower(-speed);
        robot.leftMotor.setPower(speed);
        robot.rightMotor.setPower(-speed);

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < duration)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
            idle();
        }

        goStop();
    }

    public void slideLeft(double speed, double duration) throws InterruptedException
    {
        // Slide left
        robot.armMotor.setPower(-speed);
        robot.tableMotor.setPower(speed);
        robot.leftMotor.setPower(-speed);
        robot.rightMotor.setPower(speed);

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < duration)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
            idle();
        }

        goStop();
    }

    public void frontWheelsToRight(double speed, LightSensor sensor) throws InterruptedException
    {
        // Slide left
        robot.armMotor.setPower(speed); //Forward
        robot.rightMotor.setPower(-speed); //Backward
        while (opModeIsActive() && (sensor.getRawLightDetected() < WHITE_THRESHOLD)) {

            // Display the light level while we are looking for the line
            telemetry.addData("Light Level",  sensor.getRawLightDetected());
            telemetry.update();
            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }
        goStop();
    }

    public void frontWheelsToLeft(double speed, LightSensor sensor) throws InterruptedException
    {
        // Slide left
        robot.armMotor.setPower(-speed); //Backward
        robot.rightMotor.setPower(speed); //Forward;

        while (opModeIsActive() && (sensor.getRawLightDetected() < WHITE_THRESHOLD)) {

            // Display the light level while we are looking for the line
            telemetry.addData("Light Level",  sensor.getRawLightDetected());
            telemetry.update();
            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }

        goStop();
    }

    public void backWheelsToRight(double speed, LightSensor sensor) throws InterruptedException
    {
        robot.tableMotor.setPower(-speed);
        robot.leftMotor.setPower(speed);

        while (opModeIsActive() && (sensor.getRawLightDetected() < WHITE_THRESHOLD)) {

            // Display the light level while we are looking for the line
            telemetry.addData("Light Level",  sensor.getRawLightDetected());
            telemetry.update();
            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }

        goStop();
    }

    public void backWheelsToLeft(double speed, LightSensor sensor) throws InterruptedException
    {
        robot.tableMotor.setPower(speed);
        robot.leftMotor.setPower(-speed);

        while (opModeIsActive() && (sensor.getRawLightDetected() < WHITE_THRESHOLD)) {

            // Display the light level while we are looking for the line
            telemetry.addData("Light Level",  sensor.getRawLightDetected());
            telemetry.update();
            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }

        goStop();
    }

    public void setupVuforia()
    {
        // Setup parameters to create localizer
        parameters = new VuforiaLocalizer.Parameters(org.firstinspires.ftc.teamcode.R.id.cameraMonitorViewId);
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        vuforiaLocalizer = ClassFactory.createVuforiaLocalizer(parameters);

        // These are the vision targets that we want to use
        // The string needs to be the name of the appropriate .xml file in the assets folder
        visionTargets = vuforiaLocalizer.loadTrackablesFromAsset("FTC_2016-17");

        // Setup the target to be tracked
        target = visionTargets.get(0); // 0 corresponds to the wheels target
        target.setName("Wheels Target");
        target.setLocation(createMatrix(0, 500, 0, 90, 0, 90));

        // Set phone location on robot
        phoneLocation = createMatrix(0, 225, 0, 90, 0, 0);

        // Setup listener and inform it of phone information
        listener = (VuforiaTrackableDefaultListener) target.getListener();
        listener.setPhoneInformation(phoneLocation, parameters.cameraDirection);
    }

    // Creates a matrix for determining the locations and orientations of objects
    // Units are millimeters for x, y, and z, and degrees for u, v, and w
    public OpenGLMatrix createMatrix(float x, float y, float z, float u, float v, float w)
    {
        return OpenGLMatrix.translation(x, y, z).
                multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES, u, v, w));
    }

    // Formats a matrix into a readable string
    public String formatMatrix(OpenGLMatrix matrix)
    {
        return matrix.formatAsTransform();
    }
}
