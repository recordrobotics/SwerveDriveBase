package frc.robot.dashboard;

public final class DashboardUI {

    public static final OverviewLayout Overview = new OverviewLayout();

    private DashboardUI() {}

    public static void update() {
        Overview.update();
    }
}
