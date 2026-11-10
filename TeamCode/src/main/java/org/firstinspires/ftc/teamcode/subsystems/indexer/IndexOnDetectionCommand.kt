package org.firstinspires.ftc.teamcode.subsystems.indexer

import com.qualcomm.robotcore.util.ElapsedTime
import com.seattlesolvers.solverslib.command.CommandBase
import com.seattlesolvers.solverslib.command.SubsystemBase
import org.firstinspires.ftc.teamcode.utils.units.Time
import java.util.function.BooleanSupplier

class IndexOnDetectionCommand(
    private val ballDetected: BooleanSupplier,
    private val pulseTime: Time,
    private val startIndexer: Runnable,
    private val stopIndexer: Runnable,
    vararg requirements: SubsystemBase
    ): CommandBase() {

    /** The index states */
    private enum class IndexState { WAITING_FOR_BALL, INDEXING, WAITING_FOR_CLEAR }

    private var state = IndexState.WAITING_FOR_BALL
    // Keeping track of the index time
    private val timer = ElapsedTime()

    init {
        // Command's requirements
        addRequirements(*requirements)
    }

    override fun initialize() {
        // First state: Waiting for a ball to come into the indexer
        state = IndexState.WAITING_FOR_BALL
    }

    override fun execute() {
        when (state) {
            // Not until a ball is detected the indexer will run.
            IndexState.WAITING_FOR_BALL -> if (ballDetected.asBoolean) {
                startIndexer.run()
                timer.reset()
                // A new state is set: Indexing
                state = IndexState.INDEXING
            }
            // Wait until the preset index time is met
            IndexState.INDEXING -> if (timer.milliseconds() >= pulseTime.millis) {
                // Once the time is met, the indexer is stopped.
                stopIndexer.run()
                // A new state is set: Waiting for clear
                state = IndexState.WAITING_FOR_CLEAR
            }
            // Re-arm only once the sensor stops seeing a ball, so one ball can't trigger twice
            IndexState.WAITING_FOR_CLEAR -> if (!ballDetected.asBoolean) {
                // The initial state is set again.
                state = IndexState.WAITING_FOR_BALL
            }
        }
    }

    override fun end(interrupted: Boolean) {
        stopIndexer.run()
    }

    override fun isFinished(): Boolean = false
}