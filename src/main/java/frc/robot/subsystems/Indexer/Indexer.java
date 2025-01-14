// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Indexer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {

  private IndexerIO indexerIO;
  public boolean isIntooked = false;

  /** Creates a new Indexer. */
  public Indexer(IndexerIO io) {
    indexerIO = io;
  }

  public void setIsIntooked(boolean intooked) {
    indexerIO.setIsIntooked(intooked);
  }

  public boolean getIsIntooked() {
    return indexerIO.getIsIntooked();
  }

  public double getCurrent() {
    return indexerIO.getCurrent();
  }

  public void setMotor(double indexerSpeed) {
    indexerIO.setMotor(indexerSpeed);
  }

  public void setSpeed(double speed) {
    indexerIO.setSpeed(speed);
  }

  // returns speed of the indexer
  public double getEncoderSpeed() {
    return indexerIO.getEncoderSpeed();
  }

  public void setCurrentLimit(int current) {
    indexerIO.setCurrentLimit(current);
  }

  public boolean getIsBeamBroken() {
    return indexerIO.getIsBeamBroken();
  }

  @Override
  public void periodic() {
    indexerIO.periodicUpdate();
  }

  public boolean isStalling() {
    return indexerIO.isStalling();
  }
}
