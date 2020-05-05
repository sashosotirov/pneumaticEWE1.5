/** @file Read_Analog_Buttons.ino
 *
 * Example code to read three buttons connected to analog pin A0 using
 * a resistor ladder.  The buttons are connected as follows:
 * 
 *    +5V ----+----
 *            |
 *           [ ] R = 10k
 *           [ ]
 *            |
 *    A0 -----+---- (Button0) ----> GND
 *            |
 *           [ ] R1 = 5k1
 *           [ ]
 *            |
 *            +---- (Button1) ----> GND
 *            |
 *           [ ] R2 = 15k
 *           [ ]
 *            |
 *            +---- (Button2) ----> GND
 *
 * See the library source code for instructions on how to calculate the
 * resistor values for different numbers of buttons.
 *
 *
 * (C) 2015 Ole Wolf <wolf@blazingangles.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <Analog_Buttons.h>


/* Initialize three analog buttons on pin A0. */
Analog_Buttons buttons( A0, 3 );


void setup( )
{
  Serial.begin( 115200 );

  /* Enable repeating buttons:  an additional keypress is reported for
     every 250 ms a button is held down. */
  buttons.repeat( 250 );
}


void loop( )
{
  /* Scan for keys and report the keypresses as they are detected. */
  int keypress = buttons;
  if( keypress != -1 )
  {
	Serial.print( "Keypress: " );
	Serial.println( keypress );
    
  }
}
