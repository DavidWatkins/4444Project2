package slather.vector;

import slather.sim.Cell;
import slather.sim.Move;
import slather.sim.Pherome;
import slather.sim.Point;

import java.util.*;

public class Player implements slather.sim.Player {

	private int trailLength;
	private double distanceVisible;
	private static final double MAXIMUM_MOVE = 1.0;
	private static final double THRESHOLD_DISTANCE = 2.0;
	private int side_length;
	private Random gen;

	public void init(double d, int t, int side_length) {
		this.trailLength = t;
		this.distanceVisible = d;
		this.side_length = side_length;
		this.gen = new Random();
	}

	private Vector getNormalizedVector(Point myPosition, Point otherPosition, double diameter) {
		//Check distance between cell and neighboring cell. If greater than theshold, add vector
		Vector vector = new Vector(myPosition, otherPosition);
		if(Math.abs(myPosition.x - otherPosition.x) > diameter + distanceVisible)
			vector = new Vector(vector.getX() - side_length, vector.getY());
		if(Math.abs(myPosition.y - otherPosition.y) > diameter + distanceVisible)
			vector = new Vector(vector.getX(), vector.getY() - side_length);
		return vector;
	}

	private List<Vector> getAllNeighborVectors(Cell player_cell, Set<Cell> nearby_cells, Set<Pherome> nearby_pheromes) {
		List<Vector> vectors = new ArrayList<>();
		//Get list of vectors of nearby neighbors
		Point myPosition = player_cell.getPosition();
		for(Cell c : nearby_cells) {
			Point otherPosition = c.getPosition();
			vectors.add(getNormalizedVector(myPosition, otherPosition, player_cell.getDiameter()));
		}

		//Also get list of nearby pheremones
		for(Pherome p : nearby_pheromes) {
			//Check distance between cell and neighboring cell. If greater than theshold, add vector
			if(player_cell.player != p.player) {
				Point otherPosition = p.getPosition();
				vectors.add(getNormalizedVector(myPosition, otherPosition, player_cell.getDiameter()));
			}
		}

		return vectors;
	}

	private List<Vector> validHoneycombPositions() {
		return null;
	}

	private Move getVectorBasedMove(Cell player_cell, Set<Cell> nearby_cells, Set<Pherome> nearby_pheromes) {
		Point myPosition = player_cell.getPosition();
		List<Vector> vectors = getAllNeighborVectors(player_cell, nearby_cells, nearby_pheromes);

		//Add all vectors together
		Vector finalVector = new Vector(0, 0);
		for(Vector v : vectors) {
			finalVector = finalVector.add(v);
		}

		//Get inverse of direction
		finalVector = finalVector.invert();

		//Get final destination point
		Point finalPoint = finalVector.add(myPosition);

		//Make sure point falls within MAXIMUM_MOVE
		double distance = finalPoint.distance(myPosition);
		if(distance > MAXIMUM_MOVE) {
			finalPoint = finalVector.multiply(MAXIMUM_MOVE/distance).add(myPosition);
		}

		// if all tries fail, just chill in place
		return new Move((new Vector(myPosition, finalPoint)).toPoint(), (byte)0);
	}

	public Move play(Cell player_cell, byte memory, Set<Cell> nearby_cells, Set<Pherome> nearby_pheromes) {
		if (player_cell.getDiameter() >= 2) // reproduce whenever possible
			return new Move(true, (byte)-1, (byte)-1);

		return getVectorBasedMove(player_cell, nearby_cells, nearby_pheromes);
	}

}
