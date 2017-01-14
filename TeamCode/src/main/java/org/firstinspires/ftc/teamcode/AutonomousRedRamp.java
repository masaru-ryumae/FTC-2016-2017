package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/*
 * This OpMode was written for the Vuforia Basics video. This demonstrates basic principles of
 * using Vuforia in FTC.
 */
@Autonomous(name = "Red Ramp")
public class AutonomousRedRamp extends LinearOpMode
{
    // Variables to be used for later
    VuforiaLocalizer vuforiaLocalizer;
    VuforiaLocalizer.Parameters parameters;
    VuforiaTrackables visionTargets;
    VuforiaTrackable target;
    VuforiaTrackableDefaultListener listener;

    OpenGLMatrix lastKnownLocation;
    OpenGLMatrix phoneLocation;

    public static final String VUFORIA_KEY = "AbeG4Kr/////AAAAGYlYzgmqaUCvl9MEIXe5q7VG+bqPbYy/jH9ZIRe+MDld1JqQEnOy1ljD1xH4lUXpV5DRpXq3iD0LcgRxjm+mTWaIxZ7jB2GroVg6Nn/HbRwnDNTWqC3UBjNAuGjAlUUbMg/CC69vQXKcZl11f97ly/RIj2leGI5MimrUsaTqIcNOQe6UUnkof1LFcpT+18Z7OhqWeRnJJDh9krwMceY1TNW7IwgB+vCrFp25jeQJF9cwclsnieT/NHhjdratCGWhCdU2FHe6mQiO+pjcT2X0suJPdtwomWCVqGDg6clj0A/yHUOrC5YHJ/RROMvAidn1Uo0a1OXL2nguV0jVA/rUeqIEYxTvS5OV0ePxeXlcrAhS"; // Insert your own key here

    HardwareBrainybot robot           = new HardwareBrainybot();
    private ElapsedTime runtime = new ElapsedTime();


    static final double     FORWARD_SPEED = -0.6;
    static final double     SLOWER_FORWARD_SPEED = -0.3;
    //static final double     TURN_SPEED    = 0.5;


    public void runOpMode() throws InterruptedException
    {
        setupVuforia();
        sleep(3000);

        // We don't know where the robot is, so set it to the origin
        // If we don't include this, it would be null, which would cause errors later on
        lastKnownLocation = createMatrix(0, 0, 0, 0, 0, 0);

         /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Hello Brainy Bots. Ready to run!");    //
        telemetry.update();


        waitForStart();

        // Start tracking the targets
        visionTargets.activate();

        // Turn left 45 degrees for 1.2 sec
        robot.armMotor.setPower(-FORWARD_SPEED);
        robot.tableMotor.setPower(-FORWARD_SPEED);
        //robot.leftMotor.setPower(FORWARD_SPEED);
        //robot.rightMotor.setPower(FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.2)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
            idle();
        }

        // Stop
        robot.armMotor.setPower(0);
        robot.tableMotor.setPower(0);
        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);

        // Go forward for 0.5 sec
        robot.armMotor.setPower(FORWARD_SPEED);
        robot.tableMotor.setPower(FORWARD_SPEED);
        robot.leftMotor.setPower(FORWARD_SPEED);
        robot.rightMotor.setPower(FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.5)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
            idle();
        }

        // Stop
        robot.armMotor.setPower(0);
        robot.tableMotor.setPower(0);
        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);

        // Turn left 45 degrees for 1.2 sec
        robot.armMotor.setPower(-FORWARD_SPEED);
        robot.tableMotor.setPower(-FORWARD_SPEED);
        //robot.leftMotor.setPower(FORWARD_SPEED);
        //robot.rightMotor.setPower(FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.2)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
            idle();
        }

        // Stop
        robot.armMotor.setPower(0);
        robot.tableMotor.setPower(0);
        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);

        // Go forward for 0.5 sec
        robot.armMotor.setPower(FORWARD_SPEED);
        robot.tableMotor.setPower(FORWARD_SPEED);
        robot.leftMotor.setPower(FORWARD_SPEED);
        robot.rightMotor.setPower(FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.5)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
            idle();
        }

        // Stop
        robot.armMotor.setPower(0);
        robot.tableMotor.setPower(0);
        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);


        // Release ball for 2 seconds
        robot.bottomArm.setPower(-0.5);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 2)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
            idle();
        }

        // Stop
        robot.armMotor.setPower(0);
        robot.tableMotor.setPower(0);
        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);
        robot.bottomArm.setPower(0);

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
        idle();

        while(opModeIsActive())
        {


            // Ask the listener for the latest information on where the robot is
            OpenGLMatrix latestLocation = listener.getUpdatedRobotLocation();

            // The listener will sometimes return null, so we check for that to prevent errors
            if(latestLocation != null)
                lastKnownLocation = latestLocation;

            // Send information about whether the target is visible, and where the robot is
            telemetry.addData("Tracking " + target.getName(), listener.isVisible());
            telemetry.addData("Last Known Location", formatMatrix(lastKnownLocation));

            // Send telemetry and idle to let hardware catch up
            telemetry.update();
            idle();
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
