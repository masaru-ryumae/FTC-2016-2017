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
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
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
    VuforiaLocalizer vuforiaLocalizer;
    VuforiaLocalizer.Parameters parameters;
    VuforiaTrackables visionTargets;
    VuforiaTrackable target;
    VuforiaTrackableDefaultListener listener;

    OpenGLMatrix lastKnownLocation;
    OpenGLMatrix phoneLocation;

    public static final String VUFORIA_KEY = "AbeG4Kr/////AAAAGYlYzgmqaUCvl9MEIXe5q7VG+bqPbYy/jH9ZIRe+MDld1JqQEnOy1ljD1xH4lUXpV5DRpXq3iD0LcgRxjm+mTWaIxZ7jB2GroVg6Nn/HbRwnDNTWqC3UBjNAuGjAlUUbMg/CC69vQXKcZl11f97ly/RIj2leGI5MimrUsaTqIcNOQe6UUnkof1LFcpT+18Z7OhqWeRnJJDh9krwMceY1TNW7IwgB+vCrFp25jeQJF9cwclsnieT/NHhjdratCGWhCdU2FHe6mQiO+pjcT2X0suJPdtwomWCVqGDg6clj0A/yHUOrC5YHJ/RROMvAidn1Uo0a1OXL2nguV0jVA/rUeqIEYxTvS5OV0ePxeXlcrAhS"; // Insert your own key here

    /* Declare OpMode members. */
    HardwareBrainybot robot           = new HardwareBrainybot();   // Use a Minabot's hardware
                                                               // could also use HardwarePushbotMatrix class.
    //double          clawOffset      = 0;                       // Servo mid position
    //final double    CLAW_SPEED      = 0.02 ;                   // sets rate to move servo

    @Override
    public void runOpMode() throws InterruptedException {
        double leftFrontY, leftRearY;
        double rightFrontY, rightRearY;
        double leftFrontX, leftRearX;
        double rightFrontX, rightRearX;
        boolean topArmB, topArmX;
        boolean bottomArmA, bottomArmY;
        double maxY, maxX;

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

            topArmB = gamepad2.b;
            bottomArmA = gamepad2.a;
            topArmX = gamepad2.x;
            bottomArmY = gamepad2.y;

            // Ask the listener for the latest information on where the robot is
            OpenGLMatrix latestLocation = listener.getUpdatedRobotLocation();

            // The listener will sometimes return null, so we check for that to prevent errors
            if(latestLocation != null)
                lastKnownLocation = latestLocation;

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
            else if ((Math.abs(leftFrontY) < 0.5) && (leftFrontX < -0.5) && (Math.abs(rightFrontY) < 0.5) && (rightFrontX < -0.5))
            {
                robot.armMotor.setPower(leftFrontX);
                robot.tableMotor.setPower(-leftFrontX);
                robot.leftMotor.setPower(rightFrontX);
                robot.rightMotor.setPower(-rightFrontX);
            }
                // Left stick, X to left, leftFronX turning forward
            else if ((Math.abs(leftFrontY) < 0.5) && (leftFrontX > 0.5) && (Math.abs(rightFrontY) < 0.5) && (rightFrontX > 0.5))
            {
                robot.armMotor.setPower(leftFrontX);
                robot.tableMotor.setPower(-leftFrontX);
                robot.leftMotor.setPower(rightFrontX);
                robot.rightMotor.setPower(-rightFrontX);
            }
            // Northeast Forward
            else if ((leftFrontX > 0.5) && (leftFrontY < -0.5) && (rightFrontX > 0.5) && (rightFrontY < -0.5)){
                robot.tableMotor.setPower(leftFrontY);
                robot.rightMotor.setPower(rightFrontY);
            }
            // Northwest Forward
            else if ((leftFrontX < -0.5) && (leftFrontY < -0.5) && (rightFrontX < -0.5) && (rightFrontY < -0.5)){
                robot.armMotor.setPower(leftFrontY);
                robot.leftMotor.setPower(rightFrontY);
            }
            // Southeast Backward
            else if ((leftFrontX > 0.5) && (leftFrontY > 0.5) && (rightFrontX > 0.5) && (rightFrontY > 0.5)){
                robot.armMotor.setPower(leftFrontY);
                robot.leftMotor.setPower(rightFrontY);
            }
            // Southwest Backward
            else if ((leftFrontX < -0.5) && (leftFrontY > 0.5) && (rightFrontX < -0.5) && (rightFrontY > 0.5)){
                robot.tableMotor.setPower(leftFrontY);
                robot.rightMotor.setPower(rightFrontY);
            }
            else if (topArmB){
                robot.topArm.setPower(1.0);
            }
            else if (topArmX){
                robot.topArm.setPower(1.0);
            }
            else if (bottomArmA){
                robot.bottomArm.setPower(-0.5);
            }
            else if (bottomArmY){
                robot.bottomArm.setPower(0.5);
            }
            else {
                robot.armMotor.setPower(0.0);
                robot.tableMotor.setPower(0.0);
                robot.leftMotor.setPower(0.0);
                robot.rightMotor.setPower(0.0);
                robot.topArm.setPower(0.0);
                robot.bottomArm.setPower(0.0);
            }

            // To see where joystick controller is and assign to correct value
            /*if ((Math.abs(leftFrontY) == 1.0) && (Math.abs(leftRearY) == 1.0)) {
                //robot.armMotor.setPower(leftFrontY);
                //robot.tableMotor.setPower(leftRearY);
                telemetry.addData("in if",  "%.2f", leftFrontY);
            }*/

            /*if(Math.abs(leftFrontY) > Math.abs(leftFrontX)) {
                robot.armMotor.setPower(leftFrontY);
            }

            else if(Math.abs(leftRearY) > Math.abs(leftRearX)) {
                robot.tableMotor.setPower(leftRearY);
            }

            else if(Math.abs(leftFrontX) > Math.abs(leftFrontY)) {
                robot.armMotor.setPower(leftFrontX);
            }

            else if(Math.abs(leftRearX) > Math.abs(leftRearY)) {
                robot.tableMotor.setPower(leftRearX);
            }*/


            // Use gamepad left & right Bumpers to open and close the claw
            /*if (gamepad1.right_bumper)
                clawOffset += CLAW_SPEED;
            else if (gamepad1.left_bumper)
                clawOffset -= CLAW_SPEED;

            // Move both servos to new position.  Assume servos are mirror image of each other.
            clawOffset = Range.clip(clawOffset, -0.5, 0.5);
            robot.leftClaw.setPosition(robot.MID_SERVO + clawOffset);
            robot.rightClaw.setPosition(robot.MID_SERVO - clawOffset);
*/
            // Use gamepad buttons to move arm up (Y) and down (A)
            // if (gamepad1.y)
            //     robot.armMotor.setPower(robot.ARM_UP_POWER);
            // else if (gamepad1.a)
            //     robot.armMotor.setPower(robot.ARM_DOWN_POWER);
            // else
            //     robot.armMotor.setPower(0.0);

            // Use gamepad buttons to move left (X) and right (B)
            // if (gamepad1.x)
            //     robot.tableMotor.setPower(robot.TABLE_LEFT_POWER);
            // else if (gamepad1.b)
            //     robot.tableMotor.setPower(robot.TABLE_RIGHT_POWER);
            // else
            //     robot.tableMotor.setPower(0.0);

            // Send telemetry message to signify robot running;
            //telemetry.addData("claw",  "Offset = %.2f", clawOffset);
            telemetry.addData("leftFrontY",  "%.2f", leftFrontY);
            //telemetry.addData("rightFrontY", "%.2f", rightFrontY);
            //telemetry.addData("leftRearY",  "%.2f", leftRearY);
            //telemetry.addData("rightRearY", "%.2f", rightRearY);
            telemetry.addData("leftFrontX",  "%.2f", leftFrontX);
            //telemetry.addData("rightFrontX", "%.2f", rightFrontX);
            //telemetry.addData("leftRearX",  "%.2f", leftRearX);
            //telemetry.addData("rightRearX", "%.2f", rightRearX);
            telemetry.addData("Tracking " + target.getName(), listener.isVisible());
            telemetry.addData("Last Known Location", formatMatrix(lastKnownLocation));

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



