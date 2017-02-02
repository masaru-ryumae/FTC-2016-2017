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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

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
@Autonomous(name = "2 - Red Beacon/Central Ball GOLD", group = "Prod")

public class AutonomousRedBeacon extends LinearOpMode {



    ColorSensor colorSensor;    // Hardware Device Object
    HardwareBrainybot robot           = new HardwareBrainybot();
    private ElapsedTime runtime = new ElapsedTime();
    OpticalDistanceSensor odsSensor;  // Hardware Device Object
    OpticalDistanceSensor odsSensorRight;  // Hardware Device Object
    LightSensor lightSensor;
    LightSensor lightSensorBack;
    LightSensor lightSensorRightFront;
    LightSensor lightSensorRightBack;

    static final double SLIDE_SPEED = -0.5; // SLIDE_SPEED always Negative value
    static final double SLOWER_SLIDE_SPEED = -0.3; // SLIDE_SPEED always Negative value
    static final double SLOWEST_SLIDE_SPEED = -0.1; // SLIDE_SPEED always Negative value
    static final double FORWARD_SPEED = -0.5;
    static final double SLOWER_FORWARD_SPEED = -0.3;
    static final double SLOWERST_FORWARD_SPEED = -0.1;
    static final double ROTATE_SPEED = -0.1;
    static final double     TURN_SPEED    = -0.5;

    static final double WHITE_THRESHOLD = 1.4;  // spans between 0.1 - 0.5 from dark to light
    static final double ODS_SENSOR_LEFT_THRESHOLD = 0.40; // Left ODS sensor threashold
    static final double ODS_SENSOR_RIGHT_THRESHOLD = 0.25; // Right ODS sensor threashold
    static final double ODS_SENSOR_COLOR_RECOG_THRESHOLD = 0.15; // Right ODS sensor threashold




    @Override
    public void runOpMode() throws InterruptedException {


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
        lightSensorRightFront = hardwareMap.lightSensor.get("light sensor right front");
        lightSensorRightBack = hardwareMap.lightSensor.get("light sensor right back");

        // Set the LED in the beginning
        colorSensor.enableLed(bLedOn);

        // turn on LED of light sensor.
        lightSensor.enableLed(true);
        lightSensorBack.enableLed(true);
        lightSensorRightFront.enableLed(true);
        lightSensorRightBack.enableLed(true);

        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Hello Brainy Bots. Ready to run!");    //
        telemetry.update();

        // After Init is pressed, this keeps updating the light values
        // GOOD, before the start, it caputures the values read.
        while (!(isStarted() || isStopRequested())) {
            // Get initial vision info
            // Ask the listener for the latest information on where the robot is
            telemetry.addData("Light Level",  lightSensor.getRawLightDetected());
            telemetry.addData("Light Back Level",  lightSensorBack.getRawLightDetected());
            telemetry.addData("Light Right Front Level",  lightSensorRightFront.getRawLightDetected());
            telemetry.addData("Light Right Back Level",  lightSensorRightBack.getRawLightDetected());
            telemetry.update();
            idle();
        }

        // wait for the start button to be pressed.
        waitForStart();

        // mecanum wheel zig/zag testing.
        goForward(FORWARD_SPEED, 2.1);
        sleep(500);
        turnLeft(FORWARD_SPEED, 0.5);
        sleep(500);
        // Approach from left of white line
        goToLineFromLeft(SLOWER_SLIDE_SPEED, lightSensor, lightSensorBack, WHITE_THRESHOLD);
        /*
        robot.armMotor.setPower(SLOWER_SLIDE_SPEED);
        robot.tableMotor.setPower(-SLOWER_SLIDE_SPEED);
        robot.leftMotor.setPower(SLOWER_SLIDE_SPEED);
        robot.rightMotor.setPower(-SLOWER_SLIDE_SPEED);

        while (opModeIsActive() && (lightSensor.getRawLightDetected() < WHITE_THRESHOLD)
                && (lightSensorBack.getRawLightDetected() < WHITE_THRESHOLD)) {

            // Display the light level while we are looking for the line
            telemetry.addData("Light Level",  lightSensor.getRawLightDetected());
            telemetry.addData("Light Back Level",  lightSensorBack.getRawLightDetected());
            telemetry.addData("Light Right Front Level",  lightSensorRightFront.getRawLightDetected());
            telemetry.addData("Light Right Back Level",  lightSensorRightBack.getRawLightDetected());
            telemetry.update();
            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }

        goStop();
        */

        // Front wheels to right
        //backWheelsToRight(SLOWER_SLIDE_SPEED, lightSensorBack);
        //frontWheelsToRight(SLOWER_SLIDE_SPEED, lightSensor);
        //turnRightTillLight(SLOWER_SLIDE_SPEED, lightSensor);
        //sleep(1000);
        // Back wheels to right
        //frontWheelsToRight(SLOWER_SLIDE_SPEED, lightSensor);
        //backWheelsToRight(SLOWER_SLIDE_SPEED, lightSensorBack);
        //sleep(5000);




        // Change to Distance Sensor
        //telemetry.addData("Status", "Switching to Distance Sensor!");
        //telemetry.update();
        //sleep(1000);


        // START: First staright move close to the button where it can recognize color
        goForwardTillDistance(SLOWERST_FORWARD_SPEED, odsSensor, ODS_SENSOR_COLOR_RECOG_THRESHOLD);
        Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);
        /*
        robot.armMotor.setPower(SLOWERST_FORWARD_SPEED);
        robot.tableMotor.setPower(SLOWERST_FORWARD_SPEED);
        robot.leftMotor.setPower(SLOWERST_FORWARD_SPEED);
        robot.rightMotor.setPower(SLOWERST_FORWARD_SPEED);

        while (opModeIsActive() && (odsSensor.getRawLightDetected() < 0.15)) { // GOOD to stop close to recognize color
            //while (odsSensor.getRawLightDetected() < 0.1) {
            telemetry.addData("ODS Raw",    odsSensor.getRawLightDetected());
            telemetry.addData("ODS Right Raw",    odsSensorRight.getRawLightDetected());
            //telemetry.addData("Status", "In first while loop");
            telemetry.update();
            //sleep(10000);
            idle();

        }

        // STOP to recognize color.
        // GOOD, without these stop, the robot does not stop from above.
        //robot.armMotor.setPower(0.0);
        //robot.tableMotor.setPower(0.0);
        //robot.leftMotor.setPower(0.0);
        //robot.rightMotor.setPower(0.0);
        goStop();
        */

        telemetry.addData("Status", "FIRST stop");
        telemetry.addData("Red  ", colorSensor.red());
        telemetry.addData("Blue ", colorSensor.blue());
        telemetry.addData("Hue ", hsvValues[0]);
        telemetry.update();
        sleep(500);
        // END: First staright move close to the button where it can recognize color

        // START: Recognize RED Color
        // Change to BLUE recognition
        //else if ((colorSensor.blue() >= 1) && (hsvValues[0] > 100) && (colorSensor.red() == 0)
        //&& (buttonPushed == false)
        //if ((colorSensor.blue() >= 1) && (hsvValues[0] >= 100) && (colorSensor.red() == 0)) {
        if ((colorSensor.red() >= 1) && (colorSensor.blue() == 0)) {
            telemetry.addData("Status", "RED DETECTED");
            telemetry.update();
            sleep(500);
            // START: Bump the button!!
            bumpButton(SLOWER_FORWARD_SPEED, odsSensor, ODS_SENSOR_LEFT_THRESHOLD);
            /*
            robot.armMotor.setPower(SLOWER_FORWARD_SPEED);
            robot.tableMotor.setPower(SLOWER_FORWARD_SPEED);
            robot.leftMotor.setPower(SLOWER_FORWARD_SPEED);
            robot.rightMotor.setPower(SLOWER_FORWARD_SPEED);

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
            */
        }
        else { //Must be Blue Color. Slide left and bump the button with right bumper.
            telemetry.addData("Status", "RED NOT DETECTED, sliding to LEFT!");
            telemetry.update();
            sleep(500);
            //slideLeft(SLIDE_SPEED, 0.5);
            goToLineFromRight(SLOWER_SLIDE_SPEED, lightSensorRightFront, lightSensorRightBack, WHITE_THRESHOLD);
            /*
            robot.armMotor.setPower(-SLOWER_SLIDE_SPEED);
            robot.tableMotor.setPower(SLOWER_SLIDE_SPEED);
            robot.leftMotor.setPower(-SLOWER_SLIDE_SPEED);
            robot.rightMotor.setPower(SLOWER_SLIDE_SPEED);

            // Make sure to line up with two right light sensors.
            while (opModeIsActive() && (lightSensorRightFront.getRawLightDetected() < WHITE_THRESHOLD)
                    && (lightSensorRightBack.getRawLightDetected() < WHITE_THRESHOLD)) {

                // Display the light level while we are looking for the line
                telemetry.addData("Light Level",  lightSensor.getRawLightDetected());
                telemetry.addData("Light Back Level",  lightSensorBack.getRawLightDetected());
                telemetry.addData("Light Right Front Level",  lightSensorRightFront.getRawLightDetected());
                telemetry.addData("Light Right Back Level",  lightSensorRightBack.getRawLightDetected());
                telemetry.update();
                idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
            }

            goStop();
            */
            sleep(500);
            // Then bump button
            //goForward(SLOWERST_FORWARD_SPEED, 0.3);
            bumpButton(SLOWER_FORWARD_SPEED, odsSensorRight, ODS_SENSOR_RIGHT_THRESHOLD);
            /*
            // START: Bump the button!!
            robot.armMotor.setPower(SLOWER_FORWARD_SPEED);
            robot.tableMotor.setPower(SLOWER_FORWARD_SPEED);
            robot.leftMotor.setPower(SLOWER_FORWARD_SPEED);
            robot.rightMotor.setPower(SLOWER_FORWARD_SPEED);

            // 0.40 is about hitting the button on beacon, if the distance sensor is right hand
            // side beacon detection, then this 0.40 is too much, about 0.20 would work.
            // odsSensorRight has a low number
            while (opModeIsActive() && (odsSensorRight.getRawLightDetected() < WHITE_THRESHOLD_ODS_SENSOR_RIGHT)) { // GOOD to stop close to recognize color
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
            */

        }
        // END: Recognize Color

        // May need to line up with the back sensors or front sensors depending on which button was
        // hit. if blue button, then line up with back sensors. If red button, then back up to line
        // up with front sensors.

        // Go back a bit after pressing the button
        telemetry.addData("Status", "Backing up");
        telemetry.update();
        sleep(500);
        goForward(-FORWARD_SPEED, 2.2);
        sleep(500);
        turnRight(FORWARD_SPEED, 0.3);
        sleep(500);

        // CODE NEEDED!!
        // Hit the center ball
        // Go up the ramp and release the ball
        /*
        turnRight(FORWARD_SPEED,0.8);
        sleep(500);
        goForward(FORWARD_SPEED, 2.3);
        sleep(500);
        handleBall(TURN_SPEED, 3.0);
        sleep(5000);
        */


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

    public void handleBall(double speed, double duration) throws InterruptedException
    {
        // This function allows robot to go forward with speed, time duration argument.

        robot.bottomArm.setPower(speed);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < duration)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
            idle();
        }

        // Stop
        goStop();
    }

    public void goForwardTillDistance(double speed, OpticalDistanceSensor sensor, double threshold) throws InterruptedException
    {
        // This function allows robot to bump the button.
        robot.armMotor.setPower(speed);
        robot.tableMotor.setPower(speed);
        robot.leftMotor.setPower(speed);
        robot.rightMotor.setPower(speed);

        while (opModeIsActive() && (sensor.getRawLightDetected() < threshold)) { // GOOD to stop close to recognize color
            //while (odsSensor.getRawLightDetected() < 0.1) {
            telemetry.addData("ODS Raw",    sensor.getRawLightDetected());
            //telemetry.addData("ODS Right Raw",    odsSensorRight.getRawLightDetected());
            //telemetry.addData("Status", "In first while loop");
            telemetry.update();
            //sleep(10000);
            idle();

        }

        // STOP to recognize color.
        // GOOD, without these stop, the robot does not stop from above.
        //robot.armMotor.setPower(0.0);
        //robot.tableMotor.setPower(0.0);
        //robot.leftMotor.setPower(0.0);
        //robot.rightMotor.setPower(0.0);
        goStop();
    }


    public void bumpButton(double speed, LightSensor sensor, double threshold) throws InterruptedException
    {
        // This function allows robot to bump the button.
        // START: Bump the button!!
        robot.armMotor.setPower(speed);
        robot.tableMotor.setPower(speed);
        robot.leftMotor.setPower(speed);
        robot.rightMotor.setPower(speed);

        // 0.40 is about hitting the button on beacon, if the distance sensor is right hand
        // side beacon detection, then this 0.40 is too much, about 0.20 would work.
        // odsSensorRight has a low number
        while (opModeIsActive() && (sensor.getRawLightDetected() < threshold)) { // GOOD to stop close to recognize color
            //while (odsSensor.getRawLightDetected() < 0.1) {
            telemetry.addData("ODS Raw",    sensor.getRawLightDetected());
            //telemetry.addData("Status", "In first while loop");
            telemetry.update();
            //sleep(10000);

        }

        // STOP to recognize color.
        // GOOD, without these stop, the robot does not stop from above.
        goStop();
        // END: Bump the button!!

    }

    public void goToLineFromLeft(double speed, LightSensor sensor1, LightSensor sensor2,
                                 double threshold) throws InterruptedException
    {
        // This function allows robot to line up to a line from left facing forward
        // START: Bump the button!!
        robot.armMotor.setPower(speed);
        robot.tableMotor.setPower(-speed);
        robot.leftMotor.setPower(speed);
        robot.rightMotor.setPower(-speed);

        while (opModeIsActive() && (sensor1.getRawLightDetected() < threshold)
                && (sensor2.getRawLightDetected() < threshold)) {

            // Display the light level while we are looking for the line
            telemetry.addData("Light Left Front Level",  sensor1.getRawLightDetected());
            telemetry.addData("Light Left Back Level",  sensor2.getRawLightDetected());
            //telemetry.addData("Light Right Front Level",  lightSensorRightFront.getRawLightDetected());
            //telemetry.addData("Light Right Back Level",  lightSensorRightBack.getRawLightDetected());
            telemetry.update();
            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }

        goStop();
    }

    public void goToLineFromRight(double speed, LightSensor sensor1, LightSensor sensor2,
                                 double threshold) throws InterruptedException
    {
        // This function allows robot to line up to a line from left facing forward
        // START: Bump the button!!
        robot.armMotor.setPower(-speed);
        robot.tableMotor.setPower(speed);
        robot.leftMotor.setPower(-speed);
        robot.rightMotor.setPower(speed);

        while (opModeIsActive() && (sensor1.getRawLightDetected() < threshold)
                && (sensor2.getRawLightDetected() < threshold)) {

            // Display the light level while we are looking for the line
            telemetry.addData("Light Left Front Level",  sensor1.getRawLightDetected());
            telemetry.addData("Light Left Back Level",  sensor2.getRawLightDetected());
            //telemetry.addData("Light Right Front Level",  lightSensorRightFront.getRawLightDetected());
            //telemetry.addData("Light Right Back Level",  lightSensorRightBack.getRawLightDetected());
            telemetry.update();
            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }

        goStop();
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

    public void turnRightTillLight(double speed, LightSensor sensor) throws InterruptedException
    {
        // This function allows robot to turn right with speed, time duration argument.
        robot.armMotor.setPower(speed);
        robot.tableMotor.setPower(speed);
        robot.leftMotor.setPower(-speed);
        robot.rightMotor.setPower(-speed);
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
