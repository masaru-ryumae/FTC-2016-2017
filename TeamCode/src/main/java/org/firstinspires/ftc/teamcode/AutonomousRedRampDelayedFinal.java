package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * This OpMode was written for the Vuforia Basics video. This demonstrates basic principles of
 * using Vuforia in FTC.
 */
@Autonomous(name = "1 - Red Ramp GOLD 10 Sec Delayed", group = "Prod")
public class AutonomousRedRampDelayedFinal extends LinearOpMode
{

    HardwareBrainybot robot           = new HardwareBrainybot();
    private ElapsedTime runtime = new ElapsedTime();


    static final double     FORWARD_SPEED = -0.6;
    static final double     SLOWER_FORWARD_SPEED = -0.3;
    static final double     TURN_SPEED    = -0.5;


    public void runOpMode() throws InterruptedException
    {
         /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Hello Brainy Bots. Ready to run!");    //
        telemetry.update();


        waitForStart();

        sleep(10000);

        // Turn left
        turnLeft(FORWARD_SPEED, 1.2);

        // Go forward
        goForward(FORWARD_SPEED, 0.5);

        // Turn left 45 degrees
        turnLeft(FORWARD_SPEED, 1.2);

        // Go forward
        goForward(FORWARD_SPEED, 0.5);

        // Release ball for 2 seconds
        handleBall(TURN_SPEED, 3.0);

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
        idle();

        while(opModeIsActive())
        {
            // Send telemetry and idle to let hardware catch up
            telemetry.update();
            idle();
        }
    }

    public void goStop()
    {
        // This function stops all the moving motors.
        robot.armMotor.setPower(0);
        robot.tableMotor.setPower(0);
        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);
        robot.bottomArm.setPower(0);
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

    public void turnRight(double speed, double duration) throws InterruptedException
    {
        // This function allows robot to turn right with speed, time duration argument.
        robot.armMotor.setPower(speed);
        robot.tableMotor.setPower(speed);
        //robot.leftMotor.setPower(speed);
        //robot.rightMotor.setPower(speed);
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
        //robot.armMotor.setPower(speed);
        //robot.tableMotor.setPower(speed);
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
}
