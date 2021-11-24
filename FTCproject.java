package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name="Driver Control", group="Strafer Testing Opmode")
public class FTCproject extends OpMode {
    HardwareMecanum robot   = new HardwareMecanum();   // Use a Mecanum's hardware




    @Override
    public void init() {
        robot.init(hardwareMap);
        robot.pulley.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.pivot_arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        //Arm settings
        robot.arm.setPosition(0.8);



    }
    @Override
    public void loop() {

        System.out.println(robot.pulley.getCurrentPosition());
        telemetry.addData("pulley position", robot.pulley.getCurrentPosition());
        telemetry.update();
        //Drivetrain Controls
        double drive = gamepad1.left_stick_y;
        double turn = gamepad1.left_stick_x;
        double strafe = gamepad1.right_stick_x;
        if(gamepad1.right_trigger>0.5){
            robot.left_drive1.setVelocity(450 * (-drive + strafe + turn));
            robot.right_drive2.setVelocity(450 * (-strafe - drive + turn));
            robot.right_drive1.setVelocity(450 * (-drive - strafe - turn));
            robot.left_drive2.setVelocity(450 * (+strafe - drive - turn));
        }else {
            robot.left_drive1.setVelocity(1600 * (-drive + strafe + turn));
            robot.right_drive2.setVelocity(1600 * (-strafe - drive + turn));
            robot.right_drive1.setVelocity(1600 * (-drive - strafe - turn));
            robot.left_drive2.setVelocity(1600 * (+strafe - drive - turn));
        }

        //Intake controls
        if (gamepad2.a){
            robot.Intake.setPower(-1);
        } else if (gamepad2.x){
            robot.Intake.setPower(1);
        }else{
            robot.Intake.setPower(0);
        }

        //Pivot arm Controls
            if (gamepad2.right_bumper){
                robot.pivot_arm.setTargetPosition(-330);
                robot.pivot_arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.pivot_arm.setVelocity(1000);
            }
        if (gamepad2.left_bumper){
                robot.pivot_arm.setTargetPosition(330);
                robot.pivot_arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.pivot_arm.setVelocity(-1000);
        }
        if (gamepad2.left_trigger>0.3){
            if (robot.pivot_arm.getCurrentPosition()<0){
                robot.pivot_arm.setTargetPosition(0);
                robot.pivot_arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.pivot_arm.setVelocity(-1000);
            }
        }
        if (gamepad2.left_trigger>0.3){
            if (robot.pivot_arm.getCurrentPosition()>0){
                robot.pivot_arm.setTargetPosition(0);
                robot.pivot_arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.pivot_arm.setVelocity(1000);
            }
        }

        //Arm Controls
        if (gamepad2.dpad_left){
            robot.arm.setPosition(robot.arm.getPosition()+0.005);
        } else if (gamepad2.dpad_right){
            robot.arm.setPosition(robot.arm.getPosition()-0.005);
        }

        //Pulley Controls
        double pulley_speed = -gamepad2.left_stick_y;
        if (gamepad2.dpad_up){
            robot.pulley.setTargetPosition(2700);
            robot.pulley.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.pulley.setVelocity(1500);
        }else if (gamepad2.dpad_down){
            robot.pulley.setTargetPosition(57);
            robot.pulley.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.pulley.setVelocity(-1500);
        }else if (robot.pulley.getCurrentPosition()>=2700) {
            robot.pulley.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.pulley.setVelocity(pulley_speed * 100);
        } else {
            robot.pulley.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.pulley.setVelocity(pulley_speed * 1500);
        }


        //Spinner Controls
        if(gamepad2.b){
            robot.left_spinner.setVelocity(-350);
            robot.right_spinner.setVelocity(-350);
        }else if(gamepad2.y){
            robot.left_spinner.setVelocity(350);
            robot.right_spinner.setVelocity(350);
        } else {
            robot.left_spinner.setVelocity(0);
            robot.right_spinner.setVelocity(0);
        }



    }
}