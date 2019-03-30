import java.awt.*;
import java.util.ArrayList;

public class Path extends Entity {
    private ArrayList<BoardPoint> points;
    private BasicStroke stroke;

    public Path(ArrayList<BoardPoint> points) {
        this.points = points;
        this.stroke = new BasicStroke(3);
    }

    @Override
    public void draw(Graphics2D g) {
        g.setColor(Color.BLACK);
        g.setStroke(stroke);
        g.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
        for(int i = 0; i < points.size() - 1; i++) {
            BoardPoint p1 = points.get(i);
            BoardPoint p2 = points.get(i + 1);

            g.drawLine((int) (p1.x * Board.PIXELS_PER_INCH) + Board.DISPLAY_MARGIN,
                    Canvas.CANVAS_HEIGHT - (int) (p1.y * Board.PIXELS_PER_INCH) - Board.DISPLAY_MARGIN,
                    (int) (p2.x * Board.PIXELS_PER_INCH) + Board.DISPLAY_MARGIN,
                    Canvas.CANVAS_HEIGHT - (int) (p2.y * Board.PIXELS_PER_INCH) - Board.DISPLAY_MARGIN);
        }
    }
}
