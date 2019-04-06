import java.awt.*;

public class Mothership extends Entity {
    private double x;
    private double y;
    private double theta;
    private BoardPoint abcApproach;
    private BoardPoint defApproach;
    private double diameter;
    private double approachDiameter;

    public Mothership(double x, double y, double theta, BoardPoint abcApproach, BoardPoint defApproach) {
        this.x = x * Board.PIXELS_PER_INCH;
        this.y = y * Board.PIXELS_PER_INCH;
        this.theta = theta;
        this.abcApproach = abcApproach;
        this.defApproach = defApproach;
        this.diameter = 25;
        this.approachDiameter = 10;
    }

    @Override
    public void draw(Graphics2D g) {
        g.setColor(Color.GREEN);

        int drawX = (int) (x + Board.DISPLAY_MARGIN - diameter/2);
        int drawY = Canvas.CANVAS_HEIGHT - (int) (y + Board.DISPLAY_MARGIN + (diameter / 2));
        g.fillOval(drawX, drawY, 25, 25);

        g.setColor(Color.BLACK);
        g.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);

        drawX = (int) (abcApproach.x * Board.PIXELS_PER_INCH + Board.DISPLAY_MARGIN - diameter/2);
        drawY = Canvas.CANVAS_HEIGHT - (int) (abcApproach.y * Board.PIXELS_PER_INCH + Board.DISPLAY_MARGIN + (this.approachDiameter / 2));
        g.fillOval(drawX, drawY, 10, 10);

        drawX = (int) (defApproach.x * Board.PIXELS_PER_INCH + Board.DISPLAY_MARGIN - diameter/2);
        drawY = Canvas.CANVAS_HEIGHT - (int) (defApproach.y * Board.PIXELS_PER_INCH + Board.DISPLAY_MARGIN + (this.approachDiameter / 2));
        g.fillOval(drawX, drawY, 10, 10);
    }
}
