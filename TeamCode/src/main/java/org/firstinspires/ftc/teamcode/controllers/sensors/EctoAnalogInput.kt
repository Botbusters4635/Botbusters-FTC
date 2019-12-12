package org.firstinspires.ftc.teamcode.controllers.sensors

import com.qualcomm.robotcore.hardware.AnalogInput
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.ValueCacher
import org.firstinspires.ftc.teamcode.core.Controller

class EctoAnalogInput(val name: String) : Controller() {
    lateinit var analogBase: AnalogInput

    override fun init(hardwareMap: HardwareMap) {
        analogBase = hardwareMap.get(AnalogInput::class.java, name)
    }

    //    private val voltageCacher = ValueCacher(0.0)
    val voltage: Double
        get() {
//            if(::analogBase.isInitialized){
//                return voltageCacher.cachedGet {
            return analogBase.voltage
//                }()
//            }else{
//                return 0.0
//            }
        }

}