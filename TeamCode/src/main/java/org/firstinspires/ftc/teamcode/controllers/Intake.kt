package org.firstinspires.ftc.teamcode.controllers

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.core.Controller

class Intake : Controller() {
    lateinit var leftIntake: DcMotor
    lateinit var rightIntake: DcMotor

    var power = 0.0
        set(value) {
            field = value
            leftIntake.power = field
            rightIntake.power = -field * 0.7
        }

    override fun init(hardwareMap: HardwareMap) {
        leftIntake = hardwareMap.get(DcMotor::class.java, "intakeLeft")
        rightIntake = hardwareMap.get(DcMotor::class.java, "intakeRight")
    }

    override fun update(timeStep: Double) {

    }
}