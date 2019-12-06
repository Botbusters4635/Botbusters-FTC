package org.firstinspires.ftc.teamcode.controllers.arm

import android.os.SystemClock
import com.qualcomm.robotcore.hardware.*
import kotlinx.coroutines.*
import org.firstinspires.ftc.teamcode.core.Controller
import org.firstinspires.ftc.teamcode.core.Coordinate
import org.firstinspires.ftc.teamcode.core.PID
import org.firstinspires.ftc.teamcode.core.PIDSettings
import kotlin.math.*


open class Arm : Controller() {
    private val lowerAnglePID = PID(PIDSettings(kP = 0.03, kI = 0.0, kD = 0.0001))
    private val upperAnglePID = PID(PIDSettings(kP = 0.015, kI = 0.0, kD = 0.0001))

    private lateinit var lowerMotor: DcMotor
    private lateinit var upperMotor: DcMotor

    private lateinit var lowerAngle: AnalogInput
    private lateinit var upperAngle: AnalogInput

    private lateinit var intakeLeft: DcMotor
    private lateinit var intakeRight: DcMotor

    var upperSpeedLimit = 0.6
    var lowerSpeedLimit = 0.4

    var targetAngles = ArmAngleValues(0.0, 0.0)

    val currentAngles: ArmAngleValues
        get() {
            val angles = ArmAngleValues()

            angles.lowerAngle = 165.0 - 49.0 * lowerAngle.voltage - 37.1 * Math.pow(lowerAngle.voltage, 2.0) + 26.1 * Math.pow(lowerAngle.voltage, 3.0) - 4.58 * Math.pow(lowerAngle.voltage, 4.0)
            val x = upperAngle.voltage
            angles.upperAngle = -1.66243031965927 * x.pow(3) - 9.090297864880776 * x.pow(2) + 130.359681271249 * x - 137.040643577643

            return angles
        }

    override fun init(hardwareMap: HardwareMap) {
        lowerMotor = hardwareMap.get(DcMotor::class.java, "lowerMotor")
        upperMotor = hardwareMap.get(DcMotor::class.java, "upperMotor")

        intakeLeft = hardwareMap.get(DcMotor::class.java, "intakeLeft")
        intakeRight = hardwareMap.get(DcMotor::class.java, "intakeRight")

        lowerAngle = hardwareMap.get(AnalogInput::class.java, "lowerAngle")
        upperAngle = hardwareMap.get(AnalogInput::class.java, "upperAngle")


        lowerMotor.direction = DcMotorSimple.Direction.REVERSE
        upperMotor.direction = DcMotorSimple.Direction.REVERSE

        intakeLeft.direction = DcMotorSimple.Direction.REVERSE
        intakeRight.direction = DcMotorSimple.Direction.REVERSE

        lowerAnglePID.target = currentAngles.lowerAngle
        upperAnglePID.target = currentAngles.upperAngle
    }

    override fun update(timeStep: Double) {

        if (currentAngles.lowerAngle < 80 && targetAngles.upperAngle < -35) {
            targetAngles.upperAngle = -35.0
        }

        if (targetAngles.lowerAngle < 35) targetAngles.lowerAngle = 35.0

        lowerAnglePID.target = targetAngles.lowerAngle
        upperAnglePID.target = targetAngles.upperAngle

        val lowerOutput = lowerAnglePID.update(currentAngles.lowerAngle, timeStep)
        val upperOutput = upperAnglePID.update(currentAngles.upperAngle, timeStep)

        //telemetry.addData("currentAngles", currentAngles)

        telemetry.addData("lowerAngle", currentAngles.lowerAngle)
        telemetry.addData("upperAngle", currentAngles.upperAngle)

        telemetry.addData("lowerError", lowerAnglePID.error)
        telemetry.addData("upperError", upperAnglePID.error)

        lowerMotor.power = if(lowerOutput.absoluteValue > lowerSpeedLimit) lowerOutput.sign * lowerSpeedLimit else lowerOutput
        upperMotor.power = if(upperOutput.absoluteValue > upperSpeedLimit) upperOutput.sign * upperSpeedLimit else upperOutput
    }
}

