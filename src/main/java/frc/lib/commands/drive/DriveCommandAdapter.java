package frc.lib.commands.drive;

/**
 * Provides default implementation for {@link DriveCommand} abstract methods
 */
public class DriveCommandAdapter extends DriveCommand {

  public DriveCommandAdapter(DriveCommandConfig config) {
    super(config);
  }

  @Override
  protected double getXSpeed() {
    return 0;
  }

  @Override
  protected double getYSpeed() {
    return 0;
  }

  @Override
  protected double getRotationSpeed() {
    return 0;
  }

  @Override
  protected boolean getFieldRelative() {
    return true;
  }

}
