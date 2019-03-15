/**
 * 
 */
package fr.ifsttar.licit.simulator.agents.perception.representation;

import java.util.ArrayList;

import fr.ifsttar.licit.simulator.agents.communication.messages.Message;

/**
 * @author Lagai
 *
 */
public class SensedVehicleAgent extends SensedVehicle{

	public ArrayList<Message> messages;
	
	public SensedVehicleAgent(long senderId, double deltaX, double deltaV, double absoluteX, double absoluteV,
			double deltaXglobal, double deltaVglobal, ArrayList<Message> messages) {
		super(senderId, deltaX, deltaV, absoluteX, absoluteV, deltaXglobal, deltaVglobal);

		this.messages = messages;
		
	}

}
