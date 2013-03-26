package cz.agents.deconfliction.vis;

import java.awt.Color;
import java.util.Random;

public class AgentColors {
    private static Color[] colors = generateColors(300);


    private static Color[] generateColors(int n) {
        Color[] colors = new Color[n];
        Random r = new Random(569112);
        for (int i=0; i<n; i++) {
            colors[i] = Color.getHSBColor(r.nextFloat(), 0.99f, 0.75f);
        }

        return colors;
    }

    public static Color getColorForAgent(int n) {
        return colors[n%colors.length];
    }
}
