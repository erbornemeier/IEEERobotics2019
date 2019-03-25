import java.awt.*;

public class Robot extends Entity {
    private double x;
    private double y;
    private double rotDegrees;
    private int diameter;

    private BasicStroke stroke;

    public Robot() {
        this(54, 54, 0);
        this.diameter = 40;
    }

    public Robot(double xInches, double yInches, double rotDegrees) {
        this.x = xInches * Board.PIXELS_PER_INCH;
        this.y = yInches * Board.PIXELS_PER_INCH;
        this.rotDegrees = rotDegrees;

        stroke = new BasicStroke(5);
    }

    public void updatePose(double xInches, double yInches, double rotDegrees) {
        this.x = xInches * Board.PIXELS_PER_INCH;
        this.y = yInches * Board.PIXELS_PER_INCH;
        this.rotDegrees = rotDegrees;
    }

    @Override
    public void draw(Graphics2D g) {
        g.setStroke(stroke);
        g.setRenderingHint(RenderingHints.KEY_ANTIALIASING,
                RenderingHints.VALUE_ANTIALIAS_ON);

        int drawX = (int)(x + Board.MARGIN - diameter/2.0);
        int drawY = Canvas.CANVAS_HEIGHT - (int)(y + Board.MARGIN + diameter/2.0);
        g.drawOval(drawX, drawY, this.diameter, this.diameter);

        int lineX = (int) (drawX + diameter/2.0);
        int lineY = (int) (drawY + diameter/2.0);
        g.drawLine(lineX, lineY,
                (int) (lineX + (diameter/2.0) * Math.cos(Math.toRadians(rotDegrees))),
                (int)(lineY - (diameter/2) * Math.sin(Math.toRadians(rotDegrees))));
    }
}
