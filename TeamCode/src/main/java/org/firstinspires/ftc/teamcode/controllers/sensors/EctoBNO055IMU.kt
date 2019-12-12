package org.firstinspires.ftc.teamcode.controllers.sensors

import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.HardwareMap
import kotlinx.coroutines.channels.Channel
import kotlinx.coroutines.runBlocking
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference
import org.firstinspires.ftc.teamcode.ValueCacher
import org.firstinspires.ftc.teamcode.core.Controller

class EctoBNO055IMU(val name: String): Controller() {
    lateinit var baseIMU: BNO055IMU
    override fun init(hardwareMap: HardwareMap) {
        baseIMU = hardwareMap.get(BNO055IMU::class.java, name)
        val parameters = BNO055IMU.Parameters()
        parameters.mode = BNO055IMU.SensorMode.IMU
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES
        baseIMU.initialize(parameters)
    }

    // BNO055IMU Heading
    private val headingCacher = ValueCacher(0.0)
    val heading: Double
        get() = headingCacher.cachedGet {
            baseIMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle.toDouble()
        }()
}