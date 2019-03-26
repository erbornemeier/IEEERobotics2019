public class PathfindingPoint extends BoardPoint {
    private boolean isBlocked;

    public PathfindingPoint(double x, double y) {
        super(x, y);

        this.isBlocked = false;
    }

    public boolean isBlocked() {
        return isBlocked;
    }

    public void setBlocked(boolean blocked) {
        isBlocked = blocked;
    }
}
