package org.firstinspires.ftc.teamcode.controllers

import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.core.Controller
import com.qualcomm.robotcore.hardware.Servo


enum class TrayHolderPosition {
    Grab,
    Release
}
class TrayHolder : Controller() {
    private lateinit var rightTrayHolder: Servo
    private lateinit var leftTrayHolder: Servo



    override fun init(hardwareMap: HardwareMap) {
        rightTrayHolder = hardwareMap.get(Servo::class.java, "rightTrayHolder")
        leftTrayHolder = hardwareMap.get(Servo::class.java, "leftTrayHolder")
    }

    override fun start() {
        setPosition(TrayHolderPosition.Release)
    }


    fun setPosition(position: TrayHolderPosition){
        when (position){
            TrayHolderPosition.Release -> {
                rightTrayHolder.position = 0.1
                leftTrayHolder.position = 1.0
            }
            TrayHolderPosition.Grab -> {
                rightTrayHolder.position = 1.0
                leftTrayHolder.position = 0.0
            }
        }
    }

    override fun update(timeStep: Double) {

    }
}