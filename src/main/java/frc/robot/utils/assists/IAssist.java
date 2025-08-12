package frc.robot.utils.assists;

public interface IAssist {

    /**
     * Apply the assist to the control
     *
     * @param control
     * @return true if the assist was applied, false otherwise
     */
    boolean apply(DrivetrainControl control);

    boolean isEnabled();

    void setEnabled(boolean enabled);
}
