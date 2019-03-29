import java.awt.*;

public class Mothership extends Entity {
    private double x;
    private double y;
    private double theta;
    private BoardPoint[] points;
    private double diameter;

    public Mothership(double x, double y, double theta, BoardPoint[] points) {
        this.x = x * Board.PIXELS_PER_INCH;
        this.y = y * Board.PIXELS_PER_INCH;
        this.theta = theta;
        this.points = points;
        this.diameter = 25;
    }

    @Override
    public void draw(Graphics2D g) {
        g.setColor(Color.GREEN);

        int drawX = (int) (x + Board.DISPLAY_MARGIN - diameter/2);
        int drawY = Canvas.CANVAS_HEIGHT - (int) (y + Board.DISPLAY_MARGIN + (diameter / 2));
        g.fillOval(drawX, drawY, 25, 25);

        g.setColor(Color.BLACK);
        g.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
        for(int i = 0; i < points.length; i++) {
            BoardPoint p1 = points[i];
            BoardPoint p2 = points[(i + 1) % points.length];

            int x1 = (int) (p1.x + Board.DISPLAY_MARGIN);
            int y1 = (int) (Canvas.CANVAS_HEIGHT - (p1.y + Board.DISPLAY_MARGIN));
            int x2 = (int) (p2.x + Board.DISPLAY_MARGIN);
            int y2 = (int) (Canvas.CANVAS_HEIGHT - (p2.y + Board.DISPLAY_MARGIN));


            g.drawLine(x1, y1, x2, y2);
        }
    }
}
