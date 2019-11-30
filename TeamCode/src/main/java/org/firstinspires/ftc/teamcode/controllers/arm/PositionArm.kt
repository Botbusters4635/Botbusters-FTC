package org.firstinspires.ftc.teamcode.controllers.arm

import android.os.SystemClock
import com.qualcomm.robotcore.hardware.AnalogInput
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Job
import kotlinx.coroutines.cancelChildren
import kotlinx.coroutines.runBlocking
import org.firstinspires.ftc.teamcode.core.Controller
import org.firstinspires.ftc.teamcode.core.Coordinate
import org.firstinspires.ftc.teamcode.core.PID
import org.firstinspires.ftc.teamcode.core.PIDSettings
import kotlin.math.*

class PositionArm : Arm() {
    private var scope = CoroutineScope(Job())

    val lowerAnglePID = PID(PIDSettings(kP = 0.045, kI = 0.001, kD = 0.0))
    val upperAnglePID = PID(PIDSettings(kP = 0.02, kI = 0.0, kD = 0.00015))

    lateinit var lowerMotor: DcMotor
    lateinit var upperMotor: DcMotor

    lateinit var lowerAngle: AnalogInput
    lateinit var upperAngle: AnalogInput

    lateinit var intakeLeft: DcMotor
    lateinit var intakeRight: DcMotor


    var clawStartedTurning = SystemClock.elapsedRealtime() / 1000.0
    var clawTurnTime = 0.5 //Seconds needed for claw to turn to position

    var targetCoordinate = ArmPosition.HOME.coordinate

    val currentCoordinate: Coordinate
        get() {
            val currentAngles = getAngles()
            return kinematics.calculateFowardKinematics(currentAngles)
        }

    val kinematics = ArmKinematics(armLenght1 = .26, armLength2 = .26)

    var currentState = ArmState.GO_TARGET

    var clawTurning = false

    var servoPosition = 0.0


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


        setClampPower(0.0)
    }

    fun moveToPosition(position: ArmPosition) {
        val currentAngles = getAngles()
        val currentCoordinate = kinematics.calculateFowardKinematics(currentAngles)


        if (position.coordinate != targetCoordinate) {
            if (currentCoordinate.x > 0.0 && position.coordinate.x < 0.0) {
                currentState = ArmState.EXCHANGE_FRONT_TO_BACK
            } else if (currentCoordinate.x < 0.0 && position.coordinate.x > 0.0) {
                currentState = ArmState.EXCHANGE_BACK_TO_FRONT
            } else {
                currentState = ArmState.GO_TARGET
            }
        }
        targetCoordinate = position.coordinate

    }

    fun getAngles(): ArmAngleValues {
        val angles = ArmAngleValues()

        angles.lowerAngle = 165.0 - 49.0 * lowerAngle.voltage - 37.1 * Math.pow(lowerAngle.voltage, 2.0) + 26.1 * Math.pow(lowerAngle.voltage, 3.0) - 4.58 * Math.pow(lowerAngle.voltage, 4.0)
        val x = upperAngle.voltage
        angles.upperAngle = -1.66243031965927 * x.pow(3) - 9.090297864880776 * x.pow(2) + 130.359681271249 * x - 137.040643577643

        return angles
    }

    override fun start() {
        turningServo.position = TURN_POS.CLOSED.value
    }

    override fun stop() {
        scope.coroutineContext.cancelChildren()
    }


    override fun update(timeStep: Double) {
        var currentTargetCoord = targetCoordinate
        when (currentState) {
            ArmState.EXCHANGE_FRONT_TO_BACK -> {
                if (currentCoordinate.closeTo(ArmPosition.EXCHANGE.coordinate)) {
                    setServoHeading(180.0)
                    if (!clawTurning) {
                        clawTurning = true
                        clawStartedTurning = SystemClock.elapsedRealtime() / 1000.0
                    } else if (SystemClock.elapsedRealtime() / 1000.0 - clawStartedTurning > clawTurnTime) {
                        clawTurning = false
                        servoPosition = 180.0
                        currentState = ArmState.GO_TARGET
                    }
                }
                currentTargetCoord = ArmPosition.EXCHANGE.coordinate
            }
            ArmState.EXCHANGE_BACK_TO_FRONT -> {
                if (currentCoordinate.closeTo(ArmPosition.EXCHANGE.coordinate)) {
                    setServoHeading(0.0)
                    if (!clawTurning) {
                        clawTurning = true
                        clawStartedTurning = SystemClock.elapsedRealtime() / 1000.0
                    } else if (SystemClock.elapsedRealtime() / 1000.0 - clawStartedTurning > clawTurnTime) {
                        clawTurning = false
                        servoPosition = 0.0
                        currentState = ArmState.GO_TARGET
                    }
                }
                currentTargetCoord = ArmPosition.EXCHANGE.coordinate
            }
            ArmState.GO_TARGET -> {
                currentTargetCoord = targetCoordinate
            }
        }


        val currentAngles = getAngles()
        val targetAngles = kinematics.calculateInverseKinematics(currentTargetCoord)

        lowerAnglePID.target = targetAngles.lowerAngle
        upperAnglePID.target = targetAngles.upperAngle

        val lowerOutput = lowerAnglePID.update(currentAngles.lowerAngle, timeStep)
        val upperOutput = upperAnglePID.update(currentAngles.upperAngle, timeStep)

        lowerMotor.power = lowerOutput
        upperMotor.power = upperOutput
    }

    fun runToPositionCommand(position: ArmPosition) = runBlocking {
        moveToPosition(position)
        while (!currentCoordinate.closeTo(position.coordinate)) {

        }
    }
}


