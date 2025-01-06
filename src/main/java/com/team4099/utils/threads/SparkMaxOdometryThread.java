package com.team4099.utils.threads;// Copyright 2021-2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.


/**
 * Provides an interface for asynchronously reading high-frequency measurements to a set of queues.
 *
 * <p>This version is intended for devices like the SparkMax that require polling rather than a
 * blocking thread. A Notifier thread is used to gather samples with consistent timing.
 */
public class SparkMaxOdometryThread {
    private java.util.List<java.util.function.DoubleSupplier> signals = new java.util.ArrayList<>();
    private java.util.List<java.util.Queue<Double>> queues = new java.util.ArrayList<>();

    private final edu.wpi.first.wpilibj.Notifier notifier;
    private static com.team4099.utils.threads.SparkMaxOdometryThread instance = null;

    public static com.team4099.utils.threads.SparkMaxOdometryThread getInstance() {
        if (instance == null) {
            instance = new com.team4099.utils.threads.SparkMaxOdometryThread();
        }
        return instance;
    }

    private SparkMaxOdometryThread() {
        notifier = new edu.wpi.first.wpilibj.Notifier(this::periodic);
        notifier.setName("SparkMaxOdometryThread");
        notifier.startPeriodic(1.0 / com.team4099.robot2025.config.constants.DrivetrainConstants.OMOMETRY_UPDATE_FREQUENCY );
    }

    public java.util.Queue<Double> registerSignal(java.util.function.DoubleSupplier signal) {
        java.util.Queue<Double> queue = new java.util.concurrent.ArrayBlockingQueue<>(100);
        com.team4099.robot2025.subsystems.drivetrain.drive.Drivetrain.Companion.setOdometryLock(true);
        try {
            signals.add(signal);
            queues.add(queue);
        } finally {
            com.team4099.robot2025.subsystems.drivetrain.drive.Drivetrain.Companion.setOdometryLock(false);
        }
        return queue;
    }

    private void periodic() {
        com.team4099.robot2025.subsystems.drivetrain.drive.Drivetrain.Companion.setOdometryLock(true);
        try {
            for (int i = 0; i < signals.size(); i++) {
                queues.get(i).offer(signals.get(i).getAsDouble());
            }
        } finally {
            com.team4099.robot2025.subsystems.drivetrain.drive.Drivetrain.Companion.setOdometryLock(false);
        }
    }
}