package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
public class HardwareMecanum {

    public DcMotorEx right_drive1;
    public DcMotorEx right_drive2;
    public DcMotorEx left_drive1;
    public DcMotorEx left_drive2;

    //Arm creation
    public Servo arm;

    //Pivot arm creation
    public DcMotorEx pivot_arm;

    //Pulley creation
    public DcMotorEx pulley;

    //Intake creation
    public CRServo Intake;

    //Spinner creation
    public DcMotorEx left_spinner;
    public DcMotorEx right_spinner;

    HardwareMap hardwareMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareMecanum(){

    }
    public void init(HardwareMap ahwMap) {
        hardwareMap = ahwMap;

        //Drivetrain hardware mapping
        right_drive1 = hardwareMap.get(DcMotorEx.class, "rightfront");
        right_drive2 = hardwareMap.get(DcMotorEx.class, "rightback");
        left_drive1 = hardwareMap.get(DcMotorEx.class, "leftfront");
        left_drive2 = hardwareMap.get(DcMotorEx.class, "leftback");

        //Arm mapping
        arm = hardwareMap.get(Servo.class, "arm");

        //Pivot arm mapping
        pivot_arm = hardwareMap.get(DcMotorEx.class, "pivot_arm");

        //Pulley mapping
        pulley = hardwareMap.get(DcMotorEx.class, "pulley");

        //Intake mapping
        Intake = hardwareMap.get(CRServo.class, "Intake");

        //Spinner mapping
        left_spinner = hardwareMap.get(DcMotorEx.class, "left_spinner");
        right_spinner = hardwareMap.get(DcMotorEx.class, "right_spinner");

        right_drive1.setVelocity(0);
        right_drive2.setVelocity(0);
        left_drive1.setVelocity(0);
        left_drive2.setVelocity(0);

        //Arm setting
        arm.setPosition(1);

        //Pivot arm settings
        pivot_arm.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        pivot_arm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        //Pulley settings
        pulley.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        pulley.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        //Drivetrain settings
        left_drive2.setDirection(DcMotorEx.Direction.REVERSE);
        left_drive1.setDirection(DcMotorEx.Direction.REVERSE);
        right_drive2.setDirection(DcMotorEx.Direction.FORWARD);

        left_drive1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        right_drive1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        right_drive2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        left_drive2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        left_drive2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        right_drive1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        right_drive2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        left_drive1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        //Spinner settings
        left_spinner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_spinner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        left_spinner.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_spinner.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
}