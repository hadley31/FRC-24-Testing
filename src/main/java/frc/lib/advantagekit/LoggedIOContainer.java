package frc.lib.advantagekit;

public interface LoggedIOContainer<T extends LoggedIO<?>> {
  public void update();

  public T getIO();
}
