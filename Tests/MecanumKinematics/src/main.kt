package ftcsim

import com.almasb.fxgl.app.GameApplication;
import com.almasb.fxgl.app.GameSettings;
import com.almasb.fxgl.dsl.FXGL;
import com.almasb.fxgl.entity.Entity;
import com.almasb.fxgl.input.UserAction;
import javafx.scene.input.KeyCode;
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.PI

class KotlinGameApp : GameApplication() {
    private var driveTrainEntity = Entity()
    private var driveTrain = DriveTrain(100.0,100.0, 20.0, 30.0)
    private var settings = GameSettings()

    override fun initSettings(settings: GameSettings) {
        settings.width = 800
        settings.height = 600
        settings.title = "FTC Sim"
        this.settings = settings
    }

    override fun initGame() {
        driveTrainEntity = FXGL.entityBuilder()
                .at( this.settings.width/2.0, this.settings.height/2.0)
                .view(driveTrain)
                .buildAndAttach()
        driveTrainEntity.rotateBy( 0.0)
    }

    override fun onUpdate(tpf: Double) {
        val update = driveTrain.update(tpf)
        val yawRadians = driveTrainEntity.rotation * PI / 180.0
        val yWorldSpeed = sin(yawRadians) * update.xSpeed + cos(yawRadians) * update.ySpeed
        val xWorldSpeed = sin(-yawRadians) * update.ySpeed + cos(-yawRadians) * update.xSpeed

        driveTrainEntity.translateY(yWorldSpeed)
        driveTrainEntity.translateX(xWorldSpeed)
        driveTrainEntity.rotateBy(update.angularSpeed)
    }

    override fun initInput() {

    }
}

fun main() {
    GameApplication.launch(KotlinGameApp::class.java, emptyArray())
}