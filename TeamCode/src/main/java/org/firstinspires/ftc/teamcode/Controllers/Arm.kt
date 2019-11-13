package org.firstinspires.ftc.teamcode.Controllers

import android.os.SystemClock
import com.qualcomm.robotcore.hardware.*
import kotlinx.coroutines.*
import org.firstinspires.ftc.teamcode.core.Controller
import org.firstinspires.ftc.teamcode.core.PID
import org.firstinspires.ftc.teamcode.core.PIDSettings
import kotlin.math.*

data class ArmMotorValues(var lowerAngle: Double = 0.00, var upperAngle: Double = 0.00)
data class ArmCoordinates(var x: Double = 0.00, var y: Double = 0.00) {
    fun closeTo(targetCoord: ArmCoordinates, nearness: Double = 10.0): Boolean {
        return this.x in (targetCoord.x - targetCoord.x * nearness / 100)..(targetCoord.x + targetCoord.x * nearness / 100) && this.y in (targetCoord.y - targetCoord.y * nearness / 100)..(targetCoord.y + targetCoord.y * nearness / 100)
    }
}


class ArmKinematics(val armLenght1: Double, val armLength2: Double) {


    fun calculateInversedKinematics(x: Double, y: Double): ArmMotorValues {
        val values = ArmMotorValues()
        var cosAngle = (x.pow(2) + y.pow(2) - armLenght1.pow(2) - armLength2.pow(2)) / (2 * armLenght1 * armLength2)

        if (cosAngle > 1) {
            cosAngle = 1.0
        } else if (cosAngle < -1.0) {
            cosAngle = -1.0
        }


        if (x > 0) {
            values.upperAngle = -acos(cosAngle)
            values.lowerAngle = atan2(y, x) + atan2((armLength2 * sin(abs(values.upperAngle))), (armLenght1 + armLength2 * cos(abs(values.upperAngle))))
        } else {
            values.upperAngle = acos(cosAngle)
            values.lowerAngle = atan2(y, x) - atan2((armLength2 * sin(abs(values.upperAngle))), (armLenght1 + armLength2 * cos(abs(values.upperAngle))))
        }
        return values

    }

    fun calculateFowardKinematics(angle1: Double, angle2: Double): ArmCoordinates {
        val values2 = ArmCoordinates()
        values2.x = cos(angle1) * armLenght1 + cos(angle1 + angle2) * armLength2
        values2.y = sin(angle1) * armLenght1 + sin(angle1 + angle2) * armLength2

        return values2
    }


}

enum class ArmState {
    TARGET_LOW, TARGET_HIGH, EXCHANGE

}

class Arm : Controller() {
    private var scope = CoroutineScope(Job())

    val lowerAnglePID = PID(PIDSettings(kP = 0.0425, kI = 0.0, kD = 0.0), scope)
    val upperAnglePID = PID(PIDSettings(kP = 0.0175, kI = 0.0, kD = 0.0001), scope)

    lateinit var lowerMotor: DcMotor
    lateinit var upperMotor: DcMotor

    lateinit var lowerAngle: AnalogInput
    lateinit var upperAngle: AnalogInput

    lateinit var intakeLeft: DcMotor
    lateinit var intakeRight: DcMotor

    private lateinit var clampServo: Servo
    private lateinit var turningServo: Servo

    val homeCoordinate: ArmCoordinates = ArmCoordinates(x = 0.20, y = 0.06)
    val topCoordinates: ArmCoordinates = ArmCoordinates(x = -0.26, y = 0.4)
    val mediumCoordinates: ArmCoordinates = ArmCoordinates(x = -0.26, y = 0.2)
    val lowCoordinates: ArmCoordinates = ArmCoordinates(x = -0.26, y = 0.007)
    val exchangeCoordinates: ArmCoordinates = ArmCoordinates(x = 0.25, y = 0.22)
    var isClawClear = false
    var isClawTurning = false
    var clawStartedTurning = SystemClock.elapsedRealtime() / 1000.0

    private var targetCoordinates: ArmCoordinates = homeCoordinate

    var originCoordinates = ArmCoordinates()

    val kinematics = ArmKinematics(armLenght1 = .26, armLength2 = .26)

    private var maxClampAngle = 0.0

    override fun init(hardwareMap: HardwareMap) {
        lowerMotor = hardwareMap.get(DcMotor::class.java, "lowerMotor")
        upperMotor = hardwareMap.get(DcMotor::class.java, "upperMotor")

        intakeLeft = hardwareMap.get(DcMotor::class.java, "intakeLeft")
        intakeRight = hardwareMap.get(DcMotor::class.java, "intakeRight")

        lowerAngle = hardwareMap.get(AnalogInput::class.java, "lowerAngle")
        upperAngle = hardwareMap.get(AnalogInput::class.java, "upperAngle")


        lowerMotor.direction = DcMotorSimple.Direction.REVERSE

        intakeLeft.direction = DcMotorSimple.Direction.REVERSE
        intakeRight.direction = DcMotorSimple.Direction.REVERSE

        clampServo = hardwareMap.get(Servo::class.java, "clampServo")
        turningServo = hardwareMap.get(Servo::class.java, "turningServo")
        clampServo.direction = Servo.Direction.REVERSE
        turningServo.direction = Servo.Direction.REVERSE
    }

    fun moveto(coordinates: ArmCoordinates) {
        if (coordinates != targetCoordinates) {
            val currentAngles = getAngles()
            originCoordinates = kinematics.calculateFowardKinematics(currentAngles.lowerAngle, currentAngles.upperAngle)
        }
        val x = coordinates.x
        val y = coordinates.y.coerceAtLeast(0.06)
        targetCoordinates = ArmCoordinates(x, y)
    }

    fun getAngles(): ArmMotorValues {
        val angles = ArmMotorValues()

        angles.lowerAngle = 165.0 - 49.0 * lowerAngle.voltage - 37.1 * Math.pow(lowerAngle.voltage, 2.0) + 26.1 * Math.pow(lowerAngle.voltage, 3.0) - 4.58 * Math.pow(lowerAngle.voltage, 4.0)
        val x = upperAngle.voltage
        angles.upperAngle = -1.66243031965927 * x.pow(3) - 9.090297864880776 * x.pow(2) + 130.359681271249 * x - 137.040643577643

        return angles
    }

    @ExperimentalCoroutinesApi
    override fun start() {
        lowerAnglePID.start()
        upperAnglePID.start()
        angleProducer()
        angleReceiver()
    }

    override fun stop() {
        scope.coroutineContext.cancelChildren()
    }

    fun angleProducer() = scope.launch {
        while (isActive) {
            val currentAngle = getAngles()
            lowerAnglePID.inputChannel.send(currentAngle.lowerAngle)
            upperAnglePID.inputChannel.send(currentAngle.upperAngle)
        }
    }

    fun angleReceiver() = scope.launch {
        var currentState = ArmState.TARGET_LOW
        var lowerTarget = 0.0
        var upperTarget = 0.0

        turningServo.position = 0.0
        while (isActive) {
            when (currentState) {
                ArmState.TARGET_LOW -> {
                    val targetAngles = kinematics.calculateInversedKinematics(targetCoordinates.x, targetCoordinates.y)
//                    telemetry.addData("targetaGNLES", targetAngles)
                    if (targetAngles.lowerAngle + targetAngles.upperAngle > 0) {
                        currentState = ArmState.EXCHANGE
                    } else {
                        lowerTarget = (targetAngles.lowerAngle * 180 / PI).coerceIn(55.0, 132.0)
                        upperTarget = (targetAngles.upperAngle * 180 / PI).coerceIn(-135.0, 134.0)
                    }
                }
                ArmState.TARGET_HIGH -> {
                    val targetAngles = kinematics.calculateInversedKinematics(targetCoordinates.x, targetCoordinates.y)
                    if (targetAngles.lowerAngle + targetAngles.upperAngle < 0) {
                        currentState = ArmState.EXCHANGE
                    } else {
                        lowerTarget = (targetAngles.lowerAngle * 180 / PI).coerceIn(55.0, 132.0)
                        upperTarget = (targetAngles.upperAngle * 180 / PI).coerceIn(-135.0, 134.0)
                    }
                }
                ArmState.EXCHANGE -> {
                    val currentAngles = getAngles()
                    val targetAngles = kinematics.calculateInversedKinematics(exchangeCoordinates.x, exchangeCoordinates.y)
                    lowerTarget = targetAngles.lowerAngle * 180 / PI
                    upperTarget = targetAngles.upperAngle * 180 / PI

                    val currentCoord = kinematics.calculateFowardKinematics(currentAngles.lowerAngle * PI / 180, currentAngles.upperAngle * PI / 180)

                    if (currentCoord.closeTo(exchangeCoordinates, 15.0)) {
                        if (!isClawTurning) {
                            isClawTurning = true
                            turningServo.position = if (turningServo.position == 1.0) 0.0 else 1.0
                            clawStartedTurning = SystemClock.elapsedRealtime() / 1000.0
                        }

                        if (SystemClock.elapsedRealtime() / 1000.0 - clawStartedTurning > 1.0 && isClawTurning) {
                            isClawTurning = false
                            currentState = if (turningServo.position == 1.0) ArmState.TARGET_HIGH else ArmState.TARGET_LOW
                        }
                    }
                }
            }
            telemetry.addData("currentState", currentState)
//            val targetAngles = kinematics.calculateInversedKinematics(targetCoordinates.x, targetCoordinates.y)
////                    telemetry.addData("targetaGNLES", targetAngles)
//            lowerTarget = (targetAngles.lowerAngle * 180 / PI).coerceIn(55.0, 132.0)
//            upperTarget = (targetAngles.upperAngle * 180 / PI).coerceIn(-135.0, 134.0)
//
//            val currentAngles = getAngles()
////
//            telemetry.addData("lowerAngle", currentAngles.lowerAngle)
//            telemetry.addData("upperAngle", currentAngles.upperAngle)


            lowerAnglePID.target = lowerTarget
            upperAnglePID.target = upperTarget

            val lowerOutput = lowerAnglePID.outputChannel.receive()
            val upperOutput = upperAnglePID.outputChannel.receive()
//            telemetry.addData("output:", lowerOutput)
//            telemetry.addData("output:", upperOutput)
            lowerMotor.power = lowerOutput
            upperMotor.power = upperOutput

//            val currentAngles = getAngles()
//
//
//            var realTargetCoordinates = targetCoordinates
//
//            if ((originCoordinates.x > 0 && targetCoordinates.x < 0) && !isClawClear) {
//
//                realTargetCoordinates = exchangeCoordinates
//

//            } else if ((originCoordinates.x < 0 && targetCoordinates.x > 0) && isClawClear) {
//
//                realTargetCoordinates = exchangeCoordinates
//
//                if (currentCoord.closeTo(exchangeCoordinates)) {
//                    if (!isClawTurning) {
//                        isClawTurning = true
//                        turningServo.position = 0.0
//                        clawStartedTurning = SystemClock.elapsedRealtime() / 1000.0
//                    }
//
//                    if (SystemClock.elapsedRealtime() / 1000.0 - clawStartedTurning > 1.0 && isClawTurning) {
//                        isClawTurning = false
//                        isClawClear = false
//                    }
//                }
//            }
//
//            val angles = kinematics.calculateInversedKinematics(realTargetCoordinates.x, realTargetCoordinates.y)
//

        }
    }


    fun intake(power: Double) {
        intakeLeft.power = power
        intakeRight.power = power
    }

    fun setClampPower(power: Double) {
        clampServo.position = power

    }

    fun setServoHeading(degrees: Double) {
        turningServo.position = degrees / 180.0
        //telemetry.addData("PositionOfTurningSevro",turningServo.position * 180 )
    }

}
// x = 0.18
// y = 0-22

