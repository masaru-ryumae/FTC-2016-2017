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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/*
 * This is an example LinearOpMode that shows how to use
 * a Modern Robotics Optical Distance Sensor
 * It assumes that the ODS sensor is configured with a name of "ods sensor".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@Autonomous(name = "MR ODS Straight Test", group = "Sensor")
//@Disabled
public class OpticalDistanceStraightTest extends LinearOpMode {

  OpticalDistanceSensor odsSensor;  // Hardware Device Object
  HardwareBrainybot robot           = new HardwareBrainybot();
  private ElapsedTime runtime = new ElapsedTime();

  static final double     FORWARD_SPEED = -0.1;
  static final double     SLOWER_FORWARD_SPEED = -0.3;

  @Override
  public void runOpMode() throws InterruptedException {

    // get a reference to our Light Sensor object.
    odsSensor = hardwareMap.opticalDistanceSensor.get("ods");

    robot.init(hardwareMap);

    // Send telemetry message to signify robot waiting;
    telemetry.addData("Status", "Hello Brainy Bots. Ready to run!");    //
    telemetry.update();

    // wait for the start button to be pressed.
    waitForStart();

    // Go forward (until the distance is close)
    robot.armMotor.setPower(FORWARD_SPEED);
    robot.tableMotor.setPower(FORWARD_SPEED);
    robot.leftMotor.setPower(FORWARD_SPEED);
    robot.rightMotor.setPower(FORWARD_SPEED);

    // while the op mode is active, loop and read the light levels.
    // Note we use opModeIsActive() as our loop condition because it is an interruptible method.
    while (opModeIsActive()) {

      // send the info back to driver station using telemetry function.
      telemetry.addData("Raw",    odsSensor.getRawLightDetected());
      telemetry.addData("Normal", odsSensor.getLightDetected());

      if (odsSensor.getLightDetected() > 0.03) {
        robot.armMotor.setPower(0.0);
        robot.tableMotor.setPower(0.0);
        robot.leftMotor.setPower(0.0);
        robot.rightMotor.setPower(0.0);
      }


      telemetry.update();
      idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
    }
  }
}
