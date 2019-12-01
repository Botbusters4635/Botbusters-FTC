package org.firstinspires.ftc.teamcode.controllers

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.core.Controller
import com.qualcomm.robotcore.hardware.Servo


enum class TrayHolderPosition {
    Grab,
    Release
}
class TrayHolder : Controller() {
    private lateinit var leftTrayHolder: Servo
    private lateinit var rightTrayHolder: Servo



    override fun init(hardwareMap: HardwareMap) {
        leftTrayHolder = hardwareMap.get(Servo::class.java, "leftTrayHolder")
        rightTrayHolder = hardwareMap.get(Servo::class.java, "rightTrayHolder")
    }

    fun setPosition(position: TrayHolderPosition){
        when (position){
            TrayHolderPosition.Grab -> {
                leftTrayHolder.position = 0.3
                rightTrayHolder.position = 1.0
            }
            TrayHolderPosition.Release -> {
                leftTrayHolder.position = 1.0
                rightTrayHolder.position = 0.3
            }
        }
    }

    override fun update(timeStep: Double) {

    }
}