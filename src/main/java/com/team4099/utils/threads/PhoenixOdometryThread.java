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


import edu.wpi.first.units.measure.Angle;

/**
 * Provides an interface for asynchronously reading high-frequency measurements to a set of queues.
 *
 * <p>This version is intended for Phoenix 6 devices on both the RIO and CANivore buses. When using
 * a CANivore, the thread uses the "waitForAll" blocking method to enable more consistent sampling.
 * This also allows Phoenix Pro users to benefit from lower latency between devices using CANivore
 * time synchronization.
 */
public class PhoenixOdometryThread extends Thread {
    private final java.util.concurrent.locks.Lock signalsLock =
            new java.util.concurrent.locks.ReentrantLock(); // Prevents conflicts when registering signals
    private com.ctre.phoenix6.BaseStatusSignal[] signals = new com.ctre.phoenix6.BaseStatusSignal[0];
    private final java.util.List<java.util.Queue<Double>> queues = new java.util.ArrayList<>();
    private boolean isCANFD = false;

    private static com.team4099.utils.threads.PhoenixOdometryThread instance = null;

    public static com.team4099.utils.threads.PhoenixOdometryThread getInstance() {
        if (instance == null) {
            instance = new com.team4099.utils.threads.PhoenixOdometryThread();
        }
        return instance;
    }

    private PhoenixOdometryThread() {
        setName("PhoenixOdometryThread");
        setDaemon(true);
        start();
    }

    public java.util.Queue<Double> registerSignal(com.ctre.phoenix6.hardware.ParentDevice device, com.ctre.phoenix6.StatusSignal<Angle> signal) {
        isCANFD = device.getNetwork().equals(com.team4099.robot2023.config.constants.Constants.Universal.CANIVORE_NAME);
        java.util.Queue<Double> queue = new java.util.concurrent.ArrayBlockingQueue<>(100);
        signalsLock.lock();
        try {
            com.ctre.phoenix6.BaseStatusSignal[] newSignals = new com.ctre.phoenix6.BaseStatusSignal[signals.length + 1];
            System.arraycopy(signals, 0, newSignals, 0, signals.length);
            newSignals[signals.length] = signal;
            signals = newSignals;
            queues.add(queue);

        } finally {
            signalsLock.unlock();
        }
        return queue;
    }

    @Override
    public void run() {
        while (true) {
            // Wait for updates from all signals
            signalsLock.lock();
            try {
                if (isCANFD && signals.length > 0) {
                    com.ctre.phoenix6.BaseStatusSignal.waitForAll(2.0 / com.team4099.robot2023.config.constants.DrivetrainConstants.OMOMETRY_UPDATE_FREQUENCY, signals);
                } else {
                    // "waitForAll" does not support blocking on multiple
                    // signals with a bus that is not CAN FD, regardless
                    // of Pro licensing. No reasoning for this behavior
                    // is provided by the documentation.
                    Thread.sleep((long) (1000.0 / com.team4099.robot2023.config.constants.DrivetrainConstants.OMOMETRY_UPDATE_FREQUENCY));
                    com.ctre.phoenix6.BaseStatusSignal.refreshAll(signals);
                }
            } catch (InterruptedException e) {
                e.printStackTrace();
            } finally {
                signalsLock.unlock();
            }

            // Save new data to queues
            com.team4099.robot2023.subsystems.drivetrain.drive.Drivetrain.Companion.setOdometryLock(true);
            try {
                for (int i = 0; i < signals.length; i++) {
                    queues.get(i).offer(signals[i].getValueAsDouble());
                }
            } finally {
                com.team4099.robot2023.subsystems.drivetrain.drive.Drivetrain.Companion.setOdometryLock(false);
            }
        }
    }
}