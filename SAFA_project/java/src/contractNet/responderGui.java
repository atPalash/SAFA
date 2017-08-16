package contractNet;

import javax.swing.*;
import java.awt.*;
import java.awt.event.*;


public class responderGui extends JFrame {
    private responderAgent myAgent;

    private JTextField titleField, coordField;

    responderGui(responderAgent a) {
        super(a.getLocalName());

        myAgent = a;

        JPanel p = new JPanel();
        p.setLayout(new GridLayout(2, 2));
        p.add(new JLabel("Job title:"));
        titleField = new JTextField(15);
        p.add(titleField);
        p.add(new JLabel("coordinates:"));
        coordField = new JTextField(15);
        p.add(coordField);
        getContentPane().add(p, BorderLayout.CENTER);

        JButton addButton = new JButton("Add");
        addButton.addActionListener(ev -> {
            try {
                String title = titleField.getText().trim();
                String coordinates = coordField.getText().trim();
                myAgent.updateCapability(title, coordinates);
                titleField.setText("");
                coordField.setText("");
            }
            catch (Exception e) {
                JOptionPane.showMessageDialog(responderGui.this, "Invalid values. "+e.getMessage(), "Error", JOptionPane.ERROR_MESSAGE);
            }
        });
        p = new JPanel();
        p.add(addButton);
        getContentPane().add(p, BorderLayout.SOUTH);

        // Make the agent terminate when the user closes
        // the GUI using the button on the upper right corner
        addWindowListener(new	WindowAdapter() {
            public void windowClosing(WindowEvent e) {
                myAgent.doDelete();
            }
        } );

        setResizable(false);
    }

    public void showGui() {
        pack();
        Dimension screenSize = Toolkit.getDefaultToolkit().getScreenSize();
        int centerX = (int)screenSize.getWidth() / 2;
        int centerY = (int)screenSize.getHeight() / 2;
        setLocation(centerX - getWidth() / 2, centerY - getHeight() / 2);
        super.setVisible(true);
    }
}


