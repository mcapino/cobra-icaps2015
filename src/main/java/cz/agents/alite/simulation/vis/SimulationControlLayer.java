package cz.agents.alite.simulation.vis;

import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.event.KeyEvent;
import java.awt.event.KeyListener;
import java.text.MessageFormat;

import cz.agents.alite.vis.Vis;
import cz.agents.alite.vis.layer.AbstractLayer;
import cz.agents.alite.vis.layer.VisLayer;
import cz.agents.alite.vis.layer.toggle.KeyToggleLayer;

public class SimulationControlLayer extends AbstractLayer {

	static public interface SimulationControlProvider {
		double getTime();

		boolean isRunning();

		void setRunning(boolean running);

		float getSpeed();

		void setSpeed(float f);
	}

	private final SimulationControlProvider simulationCtrl;
	private KeyListener keyListener;

	SimulationControlLayer(SimulationControlProvider simulationControlProvider) {
		this.simulationCtrl = simulationControlProvider;
	}

	@Override
	public void init(Vis vis) {
		super.init(vis);

		keyListener = new KeyListener() {

			@Override
			public void keyTyped(KeyEvent e) {
			}

			@Override
			public void keyReleased(KeyEvent e) {
			}

			@Override
			public void keyPressed(KeyEvent e) {
				if (e.getKeyChar() == '+') {
					simulationCtrl.setSpeed(simulationCtrl.getSpeed() * 0.8f);
				} else if (e.getKeyChar() == '-') {
					simulationCtrl.setSpeed(simulationCtrl.getSpeed() * 1.2f);
				} else if (e.getKeyChar() == '*') {
					simulationCtrl.setSpeed(1.0f);
				} else if (e.getKeyChar() == ' ') {
					if (simulationCtrl.isRunning()) {
						simulationCtrl.setRunning(false);
					} else {
						simulationCtrl.setRunning(true);
					}
				}
			}
		};
		vis.addKeyListener(keyListener);
	}

	@Override
	public void deinit(Vis vis) {
		super.deinit(vis);
		vis.removeKeyListener(keyListener);
	}

	@Override
	public void paint(Graphics2D canvas) {
		StringBuilder label = new StringBuilder();
		label.append("Time: ");
		label.append(String.format("%.2f", simulationCtrl.getTime()));
		label.append(" ");
		
        if (simulationCtrl.isRunning()) {
            label.append("(");
            label.append(MessageFormat.format("{0,number,#.##}", 1/simulationCtrl.getSpeed()));
            label.append("x)");
        } else {
            label.append("(PAUSED)");
        }
		
		
		
		canvas.setColor(Color.BLUE);
		canvas.drawString(label.toString(), 15, 20);
	}

	public static VisLayer create(
			SimulationControlProvider simulationCtrlProvider) {
		VisLayer simulationControl = new SimulationControlLayer(
				simulationCtrlProvider);

		KeyToggleLayer toggle = KeyToggleLayer.create("s");
		toggle.addSubLayer(simulationControl);
		toggle.setHelpOverrideString(simulationControl.getLayerDescription()
				+ "\n"
				+ "By pressing 's', the simulation info can be turned off and on.");

		return toggle;
	}
}
