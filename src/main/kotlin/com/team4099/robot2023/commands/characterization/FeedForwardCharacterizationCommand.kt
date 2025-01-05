package com.team4099.robot2023.commands.characterization

import com.team4099.robot2023.subsystems.drivetrain.drive.Drivetrain
import com.team4099.utils.PolynomialRegression
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import org.littletonrobotics.junction.Logger
import org.team4099.lib.units.LinearVelocity
import org.team4099.lib.units.base.inSeconds
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.inMetersPerSecond
import java.util.LinkedList
import java.util.function.Consumer
import java.util.function.Supplier

class FeedForwardCharacterizationCommand(
  val drivetrain: Drivetrain,
  val voltageConsumer: Consumer<ElectricalPotential>,
  val velocitySupplier: Supplier<LinearVelocity>
) : Command() {
  private val startDelay = 1.0.seconds
  private val rampRatePerSecond = 0.1.volts
  private val timer = Timer()
  private lateinit var data: FeedForwardCharacterizationData

  init {
    addRequirements(drivetrain)
  }

  override fun initialize() {
    data = FeedForwardCharacterizationData()
    timer.reset()
    timer.start()
  }

  override fun execute() {
    drivetrain.swerveModules.forEach { it.zeroSteering() }

    if (timer.get() < startDelay.inSeconds) {
      voltageConsumer.accept(0.volts)
    } else {
      val voltage = ((timer.get() - startDelay.inSeconds) * rampRatePerSecond.inVolts).volts
      voltageConsumer.accept(voltage)
      data.add(velocitySupplier.get(), voltage)
      Logger.recordOutput("Drivetrain/appliedVoltage", voltage.inVolts)
    }
  }

  override fun end(interrupted: Boolean) {
    voltageConsumer.accept(0.volts)
    timer.stop()
    data.print()
  }

  override fun isFinished(): Boolean {
    return false
  }

  companion object {
    class FeedForwardCharacterizationData {
      private val velocityData: MutableList<LinearVelocity> = LinkedList()
      private val voltageData: MutableList<ElectricalPotential> = LinkedList()
      fun add(velocity: LinearVelocity, voltage: ElectricalPotential) {
        if (velocity.absoluteValue.inMetersPerSecond > 1E-4) {
          velocityData.add(velocity.absoluteValue)
          voltageData.add(voltage.absoluteValue)
        }
      }

      fun print() {
        if (velocityData.size == 0 || voltageData.size == 0) {
          return
        }
        val regression =
          PolynomialRegression(
            velocityData
              .stream()
              .mapToDouble { obj: LinearVelocity -> obj.inMetersPerSecond }
              .toArray(),
            voltageData
              .stream()
              .mapToDouble { obj: ElectricalPotential -> obj.inVolts }
              .toArray(),
            1
          )
        println("FF Characterization Results:")
        println("\tCount=" + Integer.toString(velocityData.size) + "")
        println(String.format("\tR2=%.5f", regression.R2()))
        println(String.format("\tkS=%.5f", regression.beta(0)))
        Logger.recordOutput("Drivetrain/kSFound", regression.beta(0))
        println(String.format("\tkV=%.5f", regression.beta(1)))
        Logger.recordOutput("Drivetrain/kVFound", regression.beta(1))
      }
    }
  }
}
