import javax.swing.*;
import java.awt.*;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

public class Canvas extends JPanel implements ActionListener {
    public static final int CANVAS_WIDTH = 1600;
    public static final int CANVAS_HEIGHT = 900;

    private DisplayController displayController;

    private JButton resetButton;
    private JTextArea textArea;

    public Canvas(DisplayController displayController) {
        setPreferredSize(new Dimension(CANVAS_WIDTH, CANVAS_HEIGHT));

        this.displayController = displayController;

        resetButton = new JButton("Reset");
        resetButton.setActionCommand("reset");
        resetButton.addActionListener(this);
        resetButton.setBounds((int) (Board.LENGTH_FEET * Board.PIXELS_PER_FOOT + Board.DISPLAY_MARGIN * 2 + 10), Board.DISPLAY_MARGIN, 80, 40);
        add(resetButton);

//        textArea = new JTextArea();
//        JScrollPane scrollPane = new JScrollPane(textArea);
//        textArea.setEditable(false);
//        scrollPane.setBounds((int) (Board.LENGTH_FEET * Board.PIXELS_PER_FOOT + Board.DISPLAY_MARGIN * 2),
//                Board.DISPLAY_MARGIN + 400,
//                Canvas.CANVAS_WIDTH - (int)(Board.LENGTH_FEET * Board.PIXELS_PER_FOOT + (Board.DISPLAY_MARGIN * 3)),
//                490);
//        add(scrollPane);

//        textArea.setFont(new Font("Ubuntu", Font.PLAIN, 20));
//        textArea.setText(textArea.getText() + "ABCDEFGH\n");
//        textArea.setText(textArea.getText() + "ABCDEFGH\n");
//        textArea.setText(textArea.getText() + "ABCDEFGH\n");
//
//        System.out.println(Canvas.CANVAS_WIDTH - (int)(Board.LENGTH_FEET * Board.PIXELS_PER_FOOT + (Board.DISPLAY_MARGIN * 2)));

        setLayout(null);
    }

    @Override
    protected void paintComponent(Graphics graphics) {
        super.paintComponent(graphics);

        Graphics2D g = (Graphics2D) graphics;

        for(int i = 0; i < displayController.getEntities().size(); i++) {
            displayController.getEntities().get(i).draw(g);
        }

        if(displayController.getPathfinding() != null) {
            displayController.getPathfinding().drawPoints(g);
        }
    }

    @Override
    public void actionPerformed(ActionEvent actionEvent) {
        if(actionEvent.getActionCommand().equals("reset")) {
            displayController.reset();
        }
    }
}
