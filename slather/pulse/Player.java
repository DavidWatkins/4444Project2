package slather.pulse;

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
	private static final int THRESHOLD_CELLS = 4, CENTER_DIAMETER=10, MAX_CENTER_DIAMETER = 45;
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

	private int getNeighborPositionsWithinDistance(double distance, Cell player_cell, Set<Cell> nearby_cells) {
		int number = 0;
		for(Cell c : nearby_cells) {
			if(player_cell.distance(c) < distance)
				number++;
		}
		return number;
	}

	private Move getVectorBasedMove(Cell player_cell, Set<Cell> nearby_cells, Set<Pherome> nearby_pheromes, byte memory) {
		Point myPosition = player_cell.getPosition();
		List<Vector> vectors = getAllNeighborVectors(player_cell, nearby_cells, nearby_pheromes);

		if(getNeighborPositionsWithinDistance(distanceVisible, player_cell, nearby_cells) > THRESHOLD_CELLS) {
			memory = (byte)(memory | 2);
		}

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
		return new Move((new Vector(myPosition, finalPoint)).toPoint(), memory);
	}

	public Move play(Cell player_cell, byte memory, Set<Cell> nearby_cells, Set<Pherome> nearby_pheromes) {
		if (player_cell.getDiameter() >= 2) // reproduce whenever possible
			return new Move(true, memory, (byte)(memory & (128 + 2 + 1)));

		int neighbors = getNeighborPositionsWithinDistance(1.5, player_cell, nearby_cells);
		if(neighbors > THRESHOLD_CELLS) {
			return getVectorBasedMove(player_cell, nearby_cells, nearby_pheromes, (byte)(memory | 0x2));
		}
		else if(neighbors > 0) {
			return getVectorBasedMove(player_cell, nearby_cells, nearby_pheromes, memory);
		}

		if((memory & 2) == 2) {//Cell has seen threshold number of cells
			return getVectorBasedMove(player_cell, nearby_cells, nearby_pheromes, memory);
		}
		else if((memory & 1) == 1) { //Cell has reached the middle before
			return getPulseBasedMove(player_cell, nearby_cells, nearby_pheromes, memory);
		}
		else { //Cell has not yet reached the middle
			return getCenterMove(player_cell, nearby_cells, nearby_pheromes, memory);
		}
	}

	private Move getCenterMove(Cell player_cell, Set<Cell> nearby_cells, Set<Pherome> nearby_pheromes, byte memory) {
		Vector toCenter = getVectorFromCenter(player_cell);
		toCenter = toCenter.multiply(1.0/toCenter.length());

		if((new Vector(player_cell.getPosition(), new Point(side_length/2, side_length/2))).length() < CENTER_DIAMETER)
			memory = (byte) (memory | 1);

		return new Move(toCenter.toPoint(), memory);
	}

	private Vector getVectorFromCenter(Cell player_cell) {
		return new Vector(new Point(side_length/2, side_length/2), player_cell.getPosition());
	}

	private double getDistanceFromCenter(Cell player_cell) {
		return getVectorFromCenter(player_cell).length();
	}

	private Move getPulseBasedMove(Cell player_cell, Set<Cell> nearby_cells, Set<Pherome> nearby_pheromes, byte memory) {

		if((memory & 4) == 0)
			memory = (byte) (memory | 4 | 8);

		Vector vector = getVectorFromCenter(player_cell).normalize();

		double distanceFromCenter = getDistanceFromCenter(player_cell);

		if(distanceFromCenter >= MAX_CENTER_DIAMETER) {
			memory = (byte) (memory & 7);
		} else if(distanceFromCenter <= CENTER_DIAMETER) {
			vector = vector.invert();
			memory = (byte) (memory | 8);
		} else if((memory & 8) == (byte)8) { //Last moved outwards
			vector = vector.invert();
		} else { //Last moved inwards
			//Do nothing
		}

		if(collides(player_cell, vector.toPoint(), nearby_cells, nearby_pheromes)) { //Collides
			vector = vector.invert();
			memory = (byte) (~(memory & 8) | (memory | 7));//Invert direction
		}
		if(collides(player_cell, vector.toPoint(), nearby_cells, nearby_pheromes)) {//Collides in both directions
			return getVectorBasedMove(player_cell, nearby_cells, nearby_pheromes, memory);
		}

		return new Move(vector.toPoint(), memory);
	}

	// convert an angle (in 2-deg increments) to a vector with magnitude Cell.move_dist (max allowed movement distance)
	private Point extractVectorFromAngle(int arg) {
		double theta = Math.toRadians( 2* (double)arg );
		double dx = Cell.move_dist * Math.cos(theta);
		double dy = Cell.move_dist * Math.sin(theta);
		return new Point(dx, dy);
	}

	private int extractCardinalityFrom(Cell player_cell) {
		Vector vector = getVectorFromCenter(player_cell);
		double theta = Math.acos(vector.getX());
		double card = Math.toDegrees(theta);
		return (int) ((card / 360.0) * 32);
	}

	// check if moving player_cell by vector collides with any nearby cell or hostile pherome
	private boolean collides(Cell player_cell, Point vector, Set<Cell> nearby_cells, Set<Pherome> nearby_pheromes) {
		Iterator<Cell> cell_it = nearby_cells.iterator();
		Point destination = player_cell.getPosition().move(vector);
		while (cell_it.hasNext()) {
			Cell other = cell_it.next();
			if (destination.distance(other.getPosition()) < 0.5 * player_cell.getDiameter() + 0.5 * other.getDiameter() + 0.00011)
				return true;
		}
		return false;
	}
}

