/**
 * 
 */
package fr.ifsttar.licit.simulator.agents.communication.messages;

import java.awt.Color;
import java.util.HashMap;

import fr.ifsttar.licit.simulator.agents.communication.messages.Message;

/**
 * @author Lagai
 *
 */
public class CommunicativeMessage extends Message {


	public String subject;
	public String performative;
	public HashMap<String, Object> parameters;

	public CommunicativeMessage(String subject, String performative, HashMap<String, Object> parameters) {

		this.subject = subject;
		this.performative = performative;
		this.parameters = parameters;

	}
}
