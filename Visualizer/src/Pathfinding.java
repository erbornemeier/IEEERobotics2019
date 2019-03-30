import java.awt.*;
import java.nio.file.Path;
import java.util.ArrayList;

public class Pathfinding {
    private static final int POINT_DRAW_SIZE_PIXELS = 5;

    private double resolution;
    private double margin;
    private ArrayList<PathfindingPoint> points;

    public Pathfinding(double resolution, double margin) {
        this.resolution = resolution;
        this.margin = margin;

        createPoints();
    }

    private void createPoints() {
        int start_x = 0;
        int start_y = 0;

        while(start_x < margin) {
            start_x += resolution;
        }

        while(start_y < margin) {
            start_y += resolution;
        }

        points = new ArrayList<>();
        for(int x = start_x; x <= Board.LENGTH_INCHES - margin; x += resolution) {
            for(int y = start_y; y <= Board.LENGTH_INCHES - margin; y += resolution) {
                PathfindingPoint p = new PathfindingPoint(x, y);
                points.add(p);
            }
        }
    }

    public void drawPoints(Graphics2D g) {
        for(PathfindingPoint point : points) {
            if(point.isBlocked()) {
                g.setColor(Color.RED);
            } else {
                g.setColor(Color.GREEN.darker());
            }

            g.fillOval((int) ((point.x * Board.PIXELS_PER_INCH) - (POINT_DRAW_SIZE_PIXELS / 2.) + Board.DISPLAY_MARGIN),
                    Canvas.CANVAS_HEIGHT - (int) ((point.y * Board.PIXELS_PER_INCH) + (POINT_DRAW_SIZE_PIXELS / 2.) + Board.DISPLAY_MARGIN),
                    POINT_DRAW_SIZE_PIXELS,
                    POINT_DRAW_SIZE_PIXELS);
        }
    }

    public void updatePoint(double x, double y, boolean isBlocked) {
        for(PathfindingPoint p : points) {
            if(Math.abs(p.x - x) < 0.01 && Math.abs(p.y - y) < 0.01) {
                p.setBlocked(isBlocked);
                break;
            }
        }
    }
}
