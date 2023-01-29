package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class HardwareMapNew {
    public DcMotor Motor = null;
    public Servo servo1 = null;

    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareMapNew(){
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        Motor = hwMap.get(DcMotor.class, "TopLeft");

        Motor.setPower(0);
        Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        servo1 = hwMap.get(Servo.class, "servo1");

    }
}