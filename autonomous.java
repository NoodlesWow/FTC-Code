package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.openftc.easyopencv.OpenCvCamera;

@Autonomous(name="Pushbot: Auto Drive By Encoder", group="Pushbot")
public class autonomous extends LinearOpMode {
    HardwareMecanum robot = new HardwareMecanum();   // Use a Mecanum's hardware
    OpenCvCamera camera;
    static final double     COUNTS_PER_MOTOR_REV    = 384.5;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 3.77953 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    private ElapsedTime runtime = new ElapsedTime();




    @Override
    public void runOpMode() {
        Pipeline.Vision vision = new Pipeline.Vision(hardwareMap, telemetry);
        Pipeline.POS position = vision.detector.getPosition();
        robot.init(hardwareMap);
        vision.setPipeline();
        vision.startStreaming();
        vision.detector.getPosition();
        while (!isStarted() && !isStopRequested()) {
            position = vision.detector.getPosition();
            telemetry.addData("position", position);
            telemetry.update();
        }
        waitForStart();
        sleep(1000);
        if (position == Pipeline.POS.CENTER) {
            telemetry.addData("position", vision.detector.getPosition());
            telemetry.update();
            encoderDrive(500, 100.0, 100.0, -100.0, -100.0, 100.0, 100);
            robot.pulley.setTargetPosition(1330);
            robot.pulley.setVelocity(1500);
            while (robot.pulley.isBusy()) {
                telemetry.addData("pulley moving", robot.pulley.getCurrentPosition());
                telemetry.update();
            }
            robot.pulley.setVelocity(0);

        }else if (position == Pipeline.POS.LEFT) {
            telemetry.addData("position", vision.detector.getPosition());
            telemetry.update();
            encoderDrive(500, -100.0, -100.0, 100.0, 100.0, 100.0, 100);
            robot.pulley.setTargetPosition(2700);
            robot.pulley.setVelocity(1500);
            while (robot.pulley.isBusy()) {
                telemetry.addData("pulley moving", robot.pulley.getCurrentPosition());
                telemetry.update();
            }
        }
    }

    


 public void encoderDrive(double speed,
                           double leftInches1, double leftInches2, double rightInches1, double rightInches2,
                           double timeoutS, long pauseM) {
     int newLeft1Target;
     int newRight1Target;
     int newLeft2Target;
     int newRight2Target;

     // Ensure that the opmode is still active
     if (opModeIsActive()) {

         // Determine new target position, and pass to motor controller
         newLeft1Target = robot.left_drive1.getCurrentPosition() + (int) (leftInches1 * COUNTS_PER_INCH);
         newRight1Target = robot.right_drive1.getCurrentPosition() + (int) (rightInches1 * COUNTS_PER_INCH);
         newLeft2Target = robot.left_drive1.getCurrentPosition() + (int) (leftInches2 * COUNTS_PER_INCH);
         newRight2Target = robot.right_drive1.getCurrentPosition() + (int) (rightInches2 * COUNTS_PER_INCH);
         robot.left_drive1.setTargetPosition(newLeft1Target);
         robot.right_drive1.setTargetPosition(newRight1Target);
         robot.left_drive2.setTargetPosition(newLeft2Target);
         robot.right_drive2.setTargetPosition(newRight2Target);

         // Turn On RUN_TO_POSITION
         robot.left_drive1.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
         robot.right_drive1.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
         robot.left_drive2.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
         robot.right_drive2.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

         // reset the timeout time and start motion.
         runtime.reset();
         robot.left_drive1.setVelocity(Math.abs(speed));
         robot.right_drive1.setVelocity(Math.abs(speed));
         robot.left_drive2.setVelocity(Math.abs(speed));
         robot.right_drive2.setVelocity(Math.abs(speed));

         // keep looping while we are still active, and there is time left, and both motors are running.
         // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
         // its target position, the motion will stop.  This is "safer" in the event that the robot will
         // always end the motion as soon as possible.
         // However, if you require that BOTH motors have finished their moves before the robot continues
         // onto the next step, use (isBusy() || isBusy()) in the loop test.
         while (opModeIsActive() &&
                 (runtime.seconds() < timeoutS) &&
                 (robot.left_drive1.isBusy() && robot.right_drive1.isBusy())) {

             // Display it for the driver.
             telemetry.addData("Path1", "Running to %7d :%7d :%7d :%7d", newLeft1Target, newRight1Target, newLeft2Target, newRight2Target);
             telemetry.addData("Path2", "Running at %7d :%7d :%7d :%7d",
                     robot.left_drive1.getCurrentPosition(),
                     robot.right_drive1.getCurrentPosition(),
                     robot.left_drive2.getCurrentPosition(),
                     robot.right_drive2.getCurrentPosition());
             telemetry.update();
         }

         // Stop all motion;
         robot.left_drive1.setVelocity(0);
         robot.right_drive1.setVelocity(0);
         robot.left_drive2.setVelocity(0);
         robot.right_drive2.setVelocity(0);

         // Turn off RUN_TO_POSITION
         robot.left_drive1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
         robot.right_drive1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
         robot.left_drive2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
         robot.right_drive2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
         sleep(pauseM);   // optional pause after each move
     }
 }
}