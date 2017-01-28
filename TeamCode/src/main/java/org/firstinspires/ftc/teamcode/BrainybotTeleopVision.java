/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


/**
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 * The code is structured as a LinearOpMode
 *
 * This particular OpMode executes a POV Game style Teleop for a PushBot
 * In this mode the left stick moves the robot FWD and back, the Right stick turns left and right.
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="BrainyBots Teleop Vision", group="Mina")
//@Disabled
public class BrainybotTeleopVision extends LinearOpMode {
        LightSensor lightSensor;  // Hardware Device Object

        VuforiaLocalizer vuforiaLocalizer;
        VuforiaLocalizer.Parameters parameters;
        VuforiaTrackables visionTargets;
        VuforiaTrackable target;
        VuforiaTrackableDefaultListener listener;

        OpenGLMatrix lastKnownLocation;
        OpenGLMatrix phoneLocation;

        public static final String VUFORIA_KEY = "AbeG4Kr/////AAAAGYlYzgmqaUCvl9MEIXe5q7VG+bqPbYy/jH9ZIRe+MDld1JqQEnOy1ljD1xH4lUXpV5DRpXq3iD0LcgRxjm+mTWaIxZ7jB2GroVg6Nn/HbRwnDNTWqC3UBjNAuGjAlUUbMg/CC69vQXKcZl11f97ly/RIj2leGI5MimrUsaTqIcNOQe6UUnkof1LFcpT+18Z7OhqWeRnJJDh9krwMceY1TNW7IwgB+vCrFp25jeQJF9cwclsnieT/NHhjdratCGWhCdU2FHe6mQiO+pjcT2X0suJPdtwomWCVqGDg6clj0A/yHUOrC5YHJ/RROMvAidn1Uo0a1OXL2nguV0jVA/rUeqIEYxTvS5OV0ePxeXlcrAhS"; // Insert your own key here

    /* Declare OpMode members. */
        HardwareBrainybot robot = new HardwareBrainybot();   // Use a Minabot's hardware
        // could also use HardwarePushbotMatrix class.
        //double          clawOffset      = 0;                       // Servo mid position
        //final double    CLAW_SPEED      = 0.02 ;                   // sets rate to move servo

        @Override
        public void runOpMode ()throws InterruptedException {
            // bPrevState and bCurrState represent the previous and current state of the button.
            boolean bPrevState = false;
            boolean bCurrState = false;

            // bLedOn represents the state of the LED.
            boolean bLedOn = true;

            // get a reference to our Light Sensor object.
            lightSensor = hardwareMap.lightSensor.get("light sensor");

            // Set the LED state in the beginning.
            lightSensor.enableLed(bLedOn);

            double leftFrontY, leftRearY;
            double rightFrontY, rightRearY;
            double leftFrontX, leftRearX;
            double rightFrontX, rightRearX;
            boolean topArmB, topArmX;
            boolean bottomArmA, bottomArmY;
            double maxY, maxX;
            double robotX = 0.0;
            double robotY = 0.0;
            double robotBearing = 0.0;
            boolean leftLadder, rightLadder, servoLadder;

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
            setupVuforia();
            sleep(3000);

            // We don't know where the robot is, so set it to the origin
            // If we don't include this, it would be null, which would cause errors later on
            lastKnownLocation = createMatrix(0, 0, 0, 0, 0, 0);
            robot.init(hardwareMap);

            // Send telemetry message to signify robot waiting;
            telemetry.addData("Say", "Hello Brainy Bots.");    //
            telemetry.update();

            // Wait for the game to start (driver presses PLAY)
            waitForStart();
            visionTargets.activate();

            // run until the end of the match (driver presses STOP)
            while (opModeIsActive()) {

                // Run wheels in POV mode (note: The joystick goes negative when pushed forwards, so negate it)
                // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
                // left  = -gamepad1.left_stick_y + gamepad1.right_stick_x;
                // right = -gamepad1.left_stick_y - gamepad1.right_stick_x;
                leftFrontY = gamepad1.left_stick_y;
                //leftRearY = -gamepad1.left_stick_y;
                rightFrontY = gamepad1.right_stick_y;
                //rightRearY = gamepad1.right_stick_y;

                leftFrontX = -gamepad1.left_stick_x;
                //leftRearX = -gamepad1.left_stick_x;
                rightFrontX = -gamepad1.right_stick_x;
                //rightRearX = -gamepad1.right_stick_x;

                topArmB = gamepad1.b;
                bottomArmA = gamepad1.a;
                topArmX = gamepad1.x;
                bottomArmY = gamepad1.y;
                leftLadder = gamepad2.a;
                rightLadder = gamepad2.y;
                servoLadder = gamepad2.b;


                // Ask the listener for the latest information on where the robot is
                OpenGLMatrix latestLocation = listener.getUpdatedRobotLocation();

                // The listener will sometimes return null, so we check for that to prevent errors
                if (latestLocation != null) {
                    lastKnownLocation = latestLocation;
                    // Then you can extract the positions and angles using the getTranslation and getOrientation methods.
                    VectorF  trans = lastKnownLocation.getTranslation();
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

                // Send information about whether the target is visible, and where the robot is
                //telemetry.addData("Tracking " + target.getName(), listener.isVisible());
                //telemetry.addData("Last Known Location", formatMatrix(lastKnownLocation));

                // Send telemetry and idle to let hardware catch up
                //telemetry.update();
                //idle();


                // Normalize the values so neither exceed +/- 1.0
            /*maxY = Math.max(Math.max(Math.abs(leftFrontY), Math.abs(leftRearY)), Math.max(Math.abs(rightFrontY), Math.abs(rightRearY)));
            if (maxY > 1.0)
            {
                leftFrontY /= maxY;
                leftRearY /= maxY;
                rightFrontY /= maxY;
                rightRearY /= maxY;
            }
*/
                //robot.armMotor.setPower(leftFrontY);
            /*robot.tableMotor.setPower(leftRearY);
            robot.leftMotor.setPower(rightFrontY);
            robot.rightMotor.setPower(rightRearY);
*/
  /*          maxX = Math.max(Math.max(Math.abs(leftFrontX), Math.abs(leftRearX)), Math.max(Math.abs(rightFrontX), Math.abs(rightRearX)));
            if (maxX > 1.0)
            {
                leftFrontX /= maxX;
                leftRearX /= maxX;
                rightFrontX /= maxX;
                rightRearX /= maxX;
            }
*/
                //robot.armMotor.setPower(leftFrontX);
            /*robot.tableMotor.setPower(leftRearX);
            robot.leftMotor.setPower(rightFrontX);
            robot.rightMotor.setPower(rightRearX);
*/
                // Left stick, Y to front/back, turning forward/backward
                if ((Math.abs(leftFrontY) > 0.5) && (Math.abs(leftFrontX) < 0.5) && (Math.abs(rightFrontY) > 0.5) && (Math.abs(rightFrontX) < 0.5)) {
                    robot.armMotor.setPower(leftFrontY);
                    robot.tableMotor.setPower(leftFrontY);
                    robot.leftMotor.setPower(rightFrontY);
                    robot.rightMotor.setPower(rightFrontY);
                }
                // Left stick, X to right, leftFronX turning backward
                else if ((Math.abs(leftFrontY) < 0.5) && (leftFrontX < -0.5) && (Math.abs(rightFrontY) < 0.5) && (rightFrontX < -0.5)) {
                    robot.armMotor.setPower(leftFrontX);
                    robot.tableMotor.setPower(-leftFrontX);
                    robot.leftMotor.setPower(rightFrontX);
                    robot.rightMotor.setPower(-rightFrontX);
                }
                // Left stick, X to left, leftFronX turning forward
                else if ((Math.abs(leftFrontY) < 0.5) && (leftFrontX > 0.5) && (Math.abs(rightFrontY) < 0.5) && (rightFrontX > 0.5)) {
                    robot.armMotor.setPower(leftFrontX);
                    robot.tableMotor.setPower(-leftFrontX);
                    robot.leftMotor.setPower(rightFrontX);
                    robot.rightMotor.setPower(-rightFrontX);
                }
                // Northeast Forward
                else if ((leftFrontX > 0.5) && (leftFrontY < -0.5) && (rightFrontX > 0.5) && (rightFrontY < -0.5)) {
                    robot.tableMotor.setPower(leftFrontY);
                    robot.rightMotor.setPower(rightFrontY);
                }
                // Northwest Forward
                else if ((leftFrontX < -0.5) && (leftFrontY < -0.5) && (rightFrontX < -0.5) && (rightFrontY < -0.5)) {
                    robot.armMotor.setPower(leftFrontY);
                    robot.leftMotor.setPower(rightFrontY);
                }
                // Southeast Backward
                else if ((leftFrontX > 0.5) && (leftFrontY > 0.5) && (rightFrontX > 0.5) && (rightFrontY > 0.5)) {
                    robot.armMotor.setPower(leftFrontY);
                    robot.leftMotor.setPower(rightFrontY);
                }
                // Southwest Backward
                else if ((leftFrontX < -0.5) && (leftFrontY > 0.5) && (rightFrontX < -0.5) && (rightFrontY > 0.5)) {
                    robot.tableMotor.setPower(leftFrontY);
                    robot.rightMotor.setPower(rightFrontY);
                } else if (leftLadder) { // Move bar UP
                    robot.leftLadderArm.setPower(-1.0);
                    robot.rightLadderArm.setPower(-1.0);
                } else if (rightLadder) { // Move bar DOWN
                    robot.leftLadderArm.setPower(1.0);
                    robot.rightLadderArm.setPower(1.0);
                } else if (bottomArmA) { // Spin outward
                    robot.bottomArm.setPower(-0.5);
                } else if (bottomArmY) { // Spin inward
                    robot.bottomArm.setPower(0.5);
                } else if (servoLadder) { // Open servo to release bars
                    robot.leftClaw.setPosition(0.0);
                }
                else {
                    robot.armMotor.setPower(0.0);
                    robot.tableMotor.setPower(0.0);
                    robot.leftMotor.setPower(0.0);
                    robot.rightMotor.setPower(0.0);
                    robot.topArm.setPower(0.0);
                    robot.bottomArm.setPower(0.0);
                    robot.leftLadderArm.setPower(0.0);
                    robot.rightLadderArm.setPower(0.0);
                    robot.leftClaw.setPosition(0.3);
                }

                // check the status of the x button .
                bCurrState = gamepad1.x;

                // check for button state transitions.
                if ((bCurrState) && (bCurrState != bPrevState)) {

                    // button is transitioning to a pressed state.  Toggle LED
                    bLedOn = !bLedOn;
                    lightSensor.enableLed(bLedOn);
                }

                // update previous state variable.
                bPrevState = bCurrState;

                // Send telemetry message to signify robot running;
                //telemetry.addData("claw",  "Offset = %.2f", clawOffset);
                telemetry.addData("leftFrontY", "%.2f", leftFrontY);
                //telemetry.addData("rightFrontY", "%.2f", rightFrontY);
                //telemetry.addData("leftRearY",  "%.2f", leftRearY);
                //telemetry.addData("rightRearY", "%.2f", rightRearY);
                telemetry.addData("leftFrontX", "%.2f", leftFrontX);
                //telemetry.addData("rightFrontX", "%.2f", rightFrontX);
                //telemetry.addData("leftRearX",  "%.2f", leftRearX);
                //telemetry.addData("rightRearX", "%.2f", rightRearX);
                telemetry.addData("Tracking " + target.getName(), listener.isVisible());
                telemetry.addData("Last Known Location", formatMatrix(lastKnownLocation));
                // send the info back to driver station using telemetry function.
                telemetry.addData("LED", bLedOn ? "On" : "Off");
                telemetry.addData("Raw", lightSensor.getRawLightDetected());
                telemetry.addData("Normal", lightSensor.getLightDetected());

                telemetry.addData("Pos X ", robotX);
                telemetry.addData("Pos Y ", robotY);
                telemetry.addData("Bear  ", robotBearing);
                //  RobotLog.vv(TAG, "robot=%s", format(lastLocation));
                telemetry.addData("Pos   ", formatMatrix(lastKnownLocation));

                telemetry.update();

                // Pause for metronome tick.  40 mS each cycle = update 25 times a second.
                robot.waitForTick(40);

            }
        }

    public void setupVuforia()
    {
        // Setup parameters to create localizer
        parameters = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
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



