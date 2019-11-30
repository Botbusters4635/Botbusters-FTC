package org.firstinspires.ftc.teamcode.controllers.arm

import android.os.SystemClock
import com.qualcomm.robotcore.hardware.*
import kotlinx.coroutines.*
import org.firstinspires.ftc.teamcode.core.Controller
import org.firstinspires.ftc.teamcode.core.Coordinate
import org.firstinspires.ftc.teamcode.core.PID
import org.firstinspires.ftc.teamcode.core.PIDSettings
import kotlin.math.*

data class ArmAngleValues(var lowerAngle: Double = 0.00, var upperAngle: Double = 0.00)

enum class ArmState {
    GO_TARGET, EXCHANGE_BACK_TO_FRONT, EXCHANGE_FRONT_TO_BACK
}

enum class TURN_POS(val value: Double) {
    OPEN(0.0), CLOSED(1.0), MIDDLE(0.5)
}

enum class ArmPosition(val coordinate: Coordinate) {
    SLOW(Coordinate(-0.42, 0.2)), HOME(Coordinate(0.20, 0.08)), TOP(Coordinate(-0.26, 0.3)), MEDIUM(Coordinate(-0.26, 0.15)), LOW(Coordinate(-0.26, 0.05)), EXCHANGE(Coordinate(0.25, 0.32)), INTAKE(Coordinate(0.25, 0.28)), PASSBRIDGE(Coordinate(0.2, 0.05))
}

open class Arm : Controller() {
    private val lowerAnglePID = PID(PIDSettings(kP = 0.045, kI = 0.001, kD = 0.0))
    private val upperAnglePID = PID(PIDSettings(kP = 0.02, kI = 0.0, kD = 0.00015))

    private lateinit var lowerMotor: DcMotor
    private lateinit var upperMotor: DcMotor

    private lateinit var lowerAngle: AnalogInput
    private lateinit var upperAngle: AnalogInput

    private lateinit var intakeLeft: DcMotor
    private lateinit var intakeRight: DcMotor

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
    }

    override fun update(timeStep: Double) {
        lowerAnglePID.target = targetAngles.lowerAngle
        upperAnglePID.target = targetAngles.upperAngle

        val lowerOutput = lowerAnglePID.update(currentAngles.lowerAngle, timeStep)
        val upperOutput = upperAnglePID.update(currentAngles.upperAngle, timeStep)

        lowerMotor.power = lowerOutput
        upperMotor.power = upperOutput
    }
}

