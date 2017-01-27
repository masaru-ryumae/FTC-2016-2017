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
//@Disabled
public class SensorMRColorBeaconPushTest extends LinearOpMode {

  ColorSensor colorSensor;    // Hardware Device Object
  HardwareBrainybot robot           = new HardwareBrainybot();
  private ElapsedTime runtime = new ElapsedTime();
  OpticalDistanceSensor odsSensor;  // Hardware Device Object

  static final double     FORWARD_SPEED = -0.1;
  boolean buttonPushed = false;


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

    // Set the LED in the beginning
    colorSensor.enableLed(bLedOn);

    robot.init(hardwareMap);

    // Send telemetry message to signify robot waiting;
    telemetry.addData("Status", "Hello Brainy Bots. Ready to run!");    //
    telemetry.update();

    // After Init is pressed, this keeps updating the light values
    while (!(isStarted() || isStopRequested())) {
        telemetry.addData("ODS Raw",    odsSensor.getRawLightDetected());
        telemetry.update();
        idle();
    }

    // wait for the start button to be pressed.
    waitForStart();

    // START: First staright move close to the button where it can recognize color
      robot.armMotor.setPower(FORWARD_SPEED);
      robot.tableMotor.setPower(FORWARD_SPEED);
      robot.leftMotor.setPower(FORWARD_SPEED);
      robot.rightMotor.setPower(FORWARD_SPEED);

      while (opModeIsActive() && (odsSensor.getRawLightDetected() < 0.15)) { // GOOD to stop close to recognize color
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
      telemetry.addData("Red  ", colorSensor.red());
      telemetry.addData("Blue ", colorSensor.blue());
      telemetry.update();
      sleep(5000);
      // END: First staright move close to the button where it can recognize color

      // START: Recognize Color
      if ((colorSensor.red() >= 3) && (colorSensor.blue() == 0) &&
              (buttonPushed == false)) {
          telemetry.addData("Status", "RED DETECTED");
          telemetry.update();
          sleep(5000);
      }
      else {
          telemetry.addData("Status", "RED NOT DETECTED");
          telemetry.update();
          sleep(5000);
      }
      // END: Recognize Color
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
}
