/** @file Analog_Buttons.h
 *
 * Library to decode a number of digital buttons that are connected to 
 * a single analog input using a resistor ladder.  See the main source
 * code for documentation.
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

#ifndef _ANALOG_BUTTONS_H
#define _ANALOG_BUTTONS_H


class Analog_Buttons
{
  public:

	/**
	 * Construct the Analog_Buttons object by assigning a pin and indicating
	 * the number of buttons on the resistor ladder.
	 *
	 * @param [in] pin_number The Arduino pin number for the analog buttons,
	 *        e.g., @a A0.
	 * @param [in] number_of_buttons The number of buttons on the resistor
	 *        ladder.
	 */
    Analog_Buttons( int pin_number, int number_of_buttons );
	/**
	 * Construct the Analog_Buttons object by assigning a pin and indicating
	 * the number of buttons on the resistor ladder.  Also, set the repeat
	 * time in the constructor without requiring the programmer to use another
	 * function call.
	 *
	 * @param [in] pin_number The Arduino pin number for the analog buttons,
	 *        e.g., @a A0.
	 * @param [in] number_of_buttons The number of buttons on the resistor
	 *        ladder.
	 * @param [in] repeat_time When a button is held down for @a repeat_time
	 *        milliseconds, pretend that the user has pressed it again.
	 */
    Analog_Buttons( int pin_number, int number_of_buttons, int repeat_time );

    /**
     * Set the repeat time to the specified number of milliseconds.
     *
     * @param [in] milliseconds The number of milliseconds that should pass
     * before a key is reported to have been pressed again.
     */
    void repeat( int milliseconds )
    {
      repeat_milliseconds = milliseconds;
    }

    /**
     * Set the debounce time.  When a key is pressed, wait until the
     * electrical connection is stable in order to avoid reporting a
     * series of rapid keypresses whenever a key is pressed.
     *
     * @param [in] milliseconds Debouncing time in milliseconds.
     */
    void debounce( int milliseconds )
    {
      debounce_milliseconds = milliseconds;
    }

    /**
     * Return the current keypress, if any.  If a key is held down and
     * repeats are enabled, the function returns a keypress each time
     * the repeat period has passed.
     *
     * @return Button number, or @a -1 if no key is pressed.
     */
    operator int( )
    {
      return( read_key( ) );
    }


  private:

    enum keypad_state_t { KEYPAD_STATE_IDLE, KEYPAD_STATE_DEBOUNCING };

    void initialize( int pin_number, int number_of_buttons, int repeat_time );
    int determine_button( ) const;
    int read_key( );
    int repeat( ) const
    {
      return( repeat_milliseconds );
    }
    int debounce( ) const
    {
      return( debounce_milliseconds );
    }

    /* Configuration. */
    int pin;
    int number_buttons;
    int repeat_milliseconds;
    int debounce_milliseconds;

    /* State machine variables. */
    int last_key;
    enum keypad_state_t keypad_state;
    unsigned long debounce_timestamp;
    unsigned long repeating_timestamp;
};


#endif /* _ANALOG_BUTTONS_H */
