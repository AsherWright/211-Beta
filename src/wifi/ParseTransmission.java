/*
* @author Sean Lawlor
* @date November 3, 2011
* @class ECSE 211 - Design Principle and Methods
* 
* Modified by F.P. Ferrie
* February 28, 2014
* Changed parameters for W2014 competition
* 
* Modified by Francois OD
* November 11, 2015
* Ported to EV3 and wifi (from NXT and bluetooth)
* Changed parameters for F2015 competition
*/
package wifi;

import java.io.DataInputStream;
import java.io.IOException;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;

/*
 * Static parsers for parsing data off the communication channel
 * 
 * The order of data is defined in the Server's Transmission class
 */

public class ParseTransmission {
	
	public static TextLCD LCD = LocalEV3.get().getTextLCD();
	
	// This should only be called after verifying that there is data in the input stream
	public static Transmission parse(DataInputStream dis) {
		Transmission trans = null;
		try {

			trans = new Transmission();
			trans.startingCorner = StartCorner.lookupCorner(dis.readInt());
			ignore(dis);
			trans.homeZoneBL_X = dis.readInt();
			ignore(dis);
			trans.homeZoneBL_Y = dis.readInt();
			ignore(dis);
			trans.homeZoneTR_X = dis.readInt();
			ignore(dis);
			trans.homeZoneTR_Y = dis.readInt();
			ignore(dis);
			trans.opponentHomeZoneBL_X = dis.readInt();
			ignore(dis);
			trans.opponentHomeZoneBL_Y = dis.readInt();
			ignore(dis);
			trans.opponentHomeZoneTR_X = dis.readInt();
			ignore(dis);
			trans.opponentHomeZoneTR_Y = dis.readInt();
			ignore(dis);
			trans.dropZone_X = dis.readInt();
			ignore(dis);
			trans.dropZone_Y = dis.readInt();
			ignore(dis);
			trans.flagType = dis.readInt();
			ignore(dis);
			trans.opponentFlagType = dis.readInt();
			ignore(dis);		
			return trans;
		} catch (IOException e) {
			// failed to read transmitted data
			LCD.drawString("IO Ex", 0, 7);
			return trans;
		}
		
	}
	
	public static void ignore(DataInputStream dis) throws IOException {
		dis.readChar();
	}
	
}
