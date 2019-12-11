package org.firstinspires.ftc.teamcode.controllers.arm

import android.os.SystemClock
import com.qualcomm.robotcore.hardware.*
import org.firstinspires.ftc.teamcode.core.Controller
import org.firstinspires.ftc.teamcode.core.PID
import org.firstinspires.ftc.teamcode.core.PIDSettings
import org.firstinspires.ftc.teamcode.motionProfiles.SCurveMotionProfile
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

    // Motion profile stuff

    private var upperMotionStartTime = 0.0
    private var lowerMotionStartTime = 0.0

    private var lowerMotionProfile = generateMotionProfile(0.0)
    private var upperMotionProfile = generateMotionProfile(0.0)

    private var upperMotionActive = false
    private var lowerMotionActive = false

    // End of motion profile stuff

    var upperSpeedLimit = 1.0
    var lowerSpeedLimit = 1.0

    var upperAngleTarget = 0.0
        set(value) {
            if(value != field){
                telemetry.addData("aaa", 2)
                field = value
                upperMotionProfile = generateMotionProfile(field - currentAngles.upperAngle)
                upperMotionActive = true
                upperMotionStartTime = SystemClock.elapsedRealtime() / 1000.0
            }
        }
    var lowerAngleTarget = 0.0
        set(value) {
            if(value != field){
                telemetry.addData("bbb", 1)
                field = value
                upperMotionProfile = generateMotionProfile(field - currentAngles.upperAngle)
                lowerMotionActive = true
                lowerMotionStartTime = SystemClock.elapsedRealtime() / 1000.0
            }
        }

    private var lastTimeAnglesRead = SystemClock.elapsedRealtime() / 1000.0
    private var lastAngleReading = ArmAngleValues()

    val currentAngles: ArmAngleValues
        get() {
            if (SystemClock.elapsedRealtime() / 1000.0 - lastTimeAnglesRead > 0.02) {
                val angles = ArmAngleValues()
//
                angles.lowerAngle = 165.0 - 49.0 * lowerAngle.voltage - 37.1 * Math.pow(lowerAngle.voltage, 2.0) + 26.1 * Math.pow(lowerAngle.voltage, 3.0) - 4.58 * Math.pow(lowerAngle.voltage, 4.0)
                val x = upperAngle.voltage
                angles.upperAngle = -1.66243031965927 * x.pow(3) - 9.090297864880776 * x.pow(2) + 130.359681271249 * x - 137.040643577643

                lastAngleReading = angles
                lastTimeAnglesRead = SystemClock.elapsedRealtime() / 1000.0
            }
            return lastAngleReading
        }

    fun generateMotionProfile(throwValue: Double): SCurveMotionProfile {
        return SCurveMotionProfile(100.0, 360.0, throwValue, 1.0)
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

//        if (currentAngles.lowerAngle < 80 && upperAngleTarget < -45) {
//            upperAngleTarget = -45.0
//        }
//
//
//        if (lowerAngleTarget < 35) lowerAngleTarget = 35.0

        if (upperMotionActive) {
            val upperTimeDelta = SystemClock.elapsedRealtime() / 1000 - upperMotionStartTime
            upperAnglePID.target = upperMotionProfile.getPosition(upperTimeDelta)
            if (upperMotionProfile.getTimeNeeded() < upperTimeDelta) {
                upperMotionActive = false
            }
        }
        telemetry.addData("highstart", upperMotionStartTime)

        if (lowerMotionActive) {
            val lowerTimeDelta = SystemClock.elapsedRealtime() / 1000 - lowerMotionStartTime
            lowerAnglePID.target = lowerMotionProfile.getPosition(lowerTimeDelta)
            if (lowerMotionProfile.getTimeNeeded() < lowerTimeDelta) {
                lowerMotionActive = false
            }
        }
        telemetry.addData("lowstart", lowerMotionStartTime)

        val lowerOutput = lowerAnglePID.update(currentAngles.lowerAngle, timeStep)
        val upperOutput = upperAnglePID.update(currentAngles.upperAngle, timeStep)
//
//        telemetry.addData("lowerAngle", currentAngles.lowerAngle)
//        telemetry.addData("upperAngle", currentAngles.upperAngle)
//
//        telemetry.addData("lowerTarget", lowerAnglePID.target)
//        telemetry.addData("upperTarget", upperAnglePID.target)

//
//        telemetry.addData("lower", lowerMotionProfile.getTimeNeeded())
//        telemetry.addData("upper", upperMotionProfile.getTimeNeeded())
//        telemetry.addData("upperMotion", upperMotionActive)
//        telemetry.addData("lowerMotion", lowerMotionActive)


        val lowerFinalOutput = if (lowerOutput.absoluteValue > lowerSpeedLimit) lowerOutput.sign * lowerSpeedLimit else lowerOutput
        val upperFinalOutput = if (upperOutput.absoluteValue > upperSpeedLimit) upperOutput.sign * upperSpeedLimit else upperOutput
        writeMotors(lowerFinalOutput, upperFinalOutput)

    }

    var lastMotorWrite = SystemClock.elapsedRealtime() / 1000


    fun writeMotors(lowerPower: Double, upperPower: Double){
        if(SystemClock.elapsedRealtime() / 1000 - lastMotorWrite > 0.02){
            lowerMotor.power = lowerPower
            upperMotor.power = upperPower

            lastMotorWrite = SystemClock.elapsedRealtime() / 1000
        }

    }
}

