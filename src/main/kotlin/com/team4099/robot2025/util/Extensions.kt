package com.team4099.robot2025.util

import com.team4099.robot2025.config.constants.Constants
import edu.wpi.first.util.WPISerializable
import edu.wpi.first.util.struct.Struct
import edu.wpi.first.util.struct.StructSerializable
import org.littletonrobotics.junction.Logger

inline fun <E : Enum<E>> Logger.recordDebugOutput(key: String, value: E) {
  if (Constants.Tuning.DEBUGING_MODE) {
    Logger.recordOutput(key, value)
  }
}

inline fun <T : WPISerializable> Logger.recordDebugOutput(key: String, value: T) {
  if (Constants.Tuning.DEBUGING_MODE) {
    Logger.recordOutput(key, value)
  }
}

inline fun <T> Logger.recordDebugOutput(key: String, value: Struct<T>) {
  if (Constants.Tuning.DEBUGING_MODE) {
    Logger.recordOutput(key, value)
  }
}

inline fun <T : StructSerializable> Logger.recordDebugOutput(key: String, vararg value: T) {
  if (Constants.Tuning.DEBUGING_MODE) {
    Logger.recordOutput(key, *value)
  }
}

inline fun Logger.recordDebugOutput(key: String, value: Array<String>) {
  if (Constants.Tuning.DEBUGING_MODE) {
    Logger.recordOutput(key, value)
  }
}

inline fun Logger.recordDebugOutput(key: String, value: BooleanArray) {
  if (Constants.Tuning.DEBUGING_MODE) {
    Logger.recordOutput(key, value)
  }
}

inline fun Logger.recordDebugOutput(key: String, value: Boolean) {
  if (Constants.Tuning.DEBUGING_MODE) {
    Logger.recordOutput(key, value)
  }
}

inline fun Logger.recordDebugOutput(key: String, value: ByteArray) {
  if (Constants.Tuning.DEBUGING_MODE) {
    Logger.recordOutput(key, value)
  }
}

inline fun Logger.recordDebugOutput(key: String, value: DoubleArray) {
  if (Constants.Tuning.DEBUGING_MODE) {
    Logger.recordOutput(key, value)
  }
}

inline fun Logger.recordDebugOutput(key: String, value: Double) {
  if (Constants.Tuning.DEBUGING_MODE) {
    Logger.recordOutput(key, value)
  }
}

inline fun Logger.recordDebugOutput(key: String, value: Float) {
  if (Constants.Tuning.DEBUGING_MODE) {
    Logger.recordOutput(key, value)
  }
}

inline fun Logger.recordDebugOutput(key: String, value: FloatArray) {
  if (Constants.Tuning.DEBUGING_MODE) {
    Logger.recordOutput(key, value)
  }
}

inline fun Logger.recordDebugOutput(key: String, value: Int) {
  if (Constants.Tuning.DEBUGING_MODE) {
    Logger.recordOutput(key, value)
  }
}

inline fun Logger.recordDebugOutput(key: String, value: IntArray) {
  if (Constants.Tuning.DEBUGING_MODE) {
    Logger.recordOutput(key, value)
  }
}

inline fun Logger.recordDebugOutput(key: String, value: Long) {
  if (Constants.Tuning.DEBUGING_MODE) {
    Logger.recordOutput(key, value)
  }
}

inline fun Logger.recordDebugOutput(key: String, value: LongArray) {
  if (Constants.Tuning.DEBUGING_MODE) {
    Logger.recordOutput(key, value)
  }
}

inline fun Logger.recordDebugOutput(key: String, value: String) {
  if (Constants.Tuning.DEBUGING_MODE) {
    Logger.recordOutput(key, value)
  }
}
