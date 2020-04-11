/** @file Analog_Buttons.cpp
 *
 * Library to decode a number of digital buttons that are connected to 
 * a single analog input using a resistor ladder.  The buttons are
 * connected as follows, in this case for four buttons:
 *
 *
 *   +5V -----+----
 *            |
 *           [ ] R
 *           [ ]
 *            |
 *    A0 -----+---- (Button0) ----> GND
 *            |
 *           [ ] R1
 *           [ ]
 *            |
 *            +---- (Button1) ----> GND
 *            |
 *           [ ] R2
 *           [ ]
 *            |
 *            +---- (Button2) ----> GND
 *            |
 *           [ ] R3
 *           [ ]
 *            |
 *            +---- (Button3) ----> GND
 *
 *
 * The buttons short the resistor ladder at different positions, causing
 * the voltage at A0 to change according to which button is pressed.
 * By selecting the resistors according to the following equation, the
 * voltage V at A0 is evenly divided in multiples of 1/N, where N is the
 * number of buttons and R  is the resistor for button n:
 *                        n
 *
 *
 *                            n-1
 *                 n          ---
 *         R  =  ----- R  -   \   R                           Eq. (1)
 *          n     N-n         /    i
 *                            ---
 *                            i=1
 *
 *
 * That is, the resistor for button n is n/(N-n) times the value of the
 * uppermost resistor R, minus the sum of the other resistors above the
 * resistor for button n.  There is no resistor for button 0.
 *
 * For example, for N = 9 buttons and R chosen at 2k2, the first resistor
 * becomes 1/8 R = 275, the closest resistor value being 270 Ohms.  The
 * The second resistor becomes 2/7 R - 270 = 359, or 330 Ohms.  The third
 * resistor becomes 3/6 R - (270+330) = 500, or 470 Ohms.  And so on.
 * These values will yield voltages at A0 close to Vcc times 8/9, 7/9,
 * 6/9, etc., depending on which button is pressed.
 *
 * The following table provides suggested values for different button
 * counts:
 *
 *
 *   ---------+-------------------------------------------------
 *    Buttons |   R      R1     R2     R3     R4     R5     R6
 *   ---------+-------------------------------------------------
 *       2    |  10k    10k
 *       3    |  10k    5k1    15k
 *       4    |  10k    3k3    6k8    22k
 *       5    |  10k    2k7    3k9    8k2    27k
 *       6    |  10k    2k2    2k7    5k1    10k    33k
 *       7    |  6k8    1k2    1k5    2k2    3k9    8k2    22k
 *
 * 
 * The library is initialized with the analog pin number (in the above
 * diagram, it is A0) and the number of buttons.
 *
 * The buttons are debounced and may optionally be configured to send
 * repeated presses if they are held down.
 *
 * Buttons are read by simply assigning the object to an integer; e.g.:
 *
 *   int button = Analog_Buttons_object;
 *
 * A negative value indicates that no button is currently pressed.
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

#include <Arduino.h>
#include "Analog_Buttons.h"


/** Debounce time in milliseconds. */
static const int DEFAULT_DEBOUNCE_TIME  = 50;
/** Milliseconds before the button "repeats" if held down.  The value 0
    disables repeat. */
static const int DEFAULT_REPEAT_TIME    =  0;

/** Number of bits in the Arduino's ADC. */
static const int ADC_RESOLUTION = 10;



/**
 * Constructor for the Analog Buttons class.  The constructor sets the
 * analog pin number and the number of buttons, and initializes the
 * button repeat time to a default value.
 * 
 * @param [in] pin_number The analog pin number; e.g., A0.
 * @param [in] number_of_buttons Number of buttons in the resistor ladder.
 */
Analog_Buttons::Analog_Buttons( int pin_number, int number_of_buttons )
{
	initialize( pin_number, number_of_buttons, DEFAULT_REPEAT_TIME );
}



/**
 * Constructor for the Analog Buttons class.  The constructor sets the
 * analog pin number, the number of buttons, and the button repeat time.
 * 
 * @param [in] pin_number The analog pin number; e.g., A0.
 * @param [in] number_of_buttons Number of buttons in the resistor ladder.
 * @param [in] repeat_time When a button is held down for @a repeat_time
 *        milliseconds, pretend that the user has pressed it again.
 */
Analog_Buttons::Analog_Buttons( int pin_number, int number_of_buttons,
								int repeat_time )
{
	initialize( pin_number, number_of_buttons, repeat_time );
}



/**
 * Initialize the Analog Buttons class.  The function is called by the
 * constructors to set the analog pin number, the number of buttons, and
 * the button repeat time, and to initialize the button debounce time to
 * a default value.
 * 
 * @param [in] pin_number The analog pin number; e.g., A0.
 * @param [in] number_of_buttons Number of buttons in the resistor ladder.
 * @param [in] repeat_time When a button is held down for @a repeat_time
 *        milliseconds, pretend that the user has pressed it again.
 */
void Analog_Buttons::initialize( int pin_number, int number_of_buttons,
								 int repeat_time )
{
  /* Initialize pin number and number of buttons. */
  pin = pin_number;
  number_buttons = number_of_buttons;

  /* Initialize the repeat time and the debounce time. */
  repeat( repeat_time );
  debounce( DEFAULT_DEBOUNCE_TIME );

  /* Configure the pin as an input pin. */
  pinMode( pin, INPUT );

  /* Initialize the state machine. */
  last_key = -1;
  keypad_state = KEYPAD_STATE_IDLE;
  debounce_timestamp = repeating_timestamp = millis( );
}



/**
 * Read the analog voltage of the button pin and determine which button
 * has been pressed based on the voltage divider output.  Because the
 * voltage divider divides the voltage in multiples of 1/N, where N is the
 * number of buttons, it becomes straight-forward to determine which
 * button is pressed by simply taking the reciprocal of the voltage and
 * rounding to the nearest integer.  (For performance reasons, these
 * calculations are performed using integer math.)
 * 
 * @return Button number, or @a -1 if no button is pressed.
 */
int Analog_Buttons::determine_button( ) const
{
  const int adc_resolution = ADC_RESOLUTION;
  const int adc_limit      = (int)( 1 << adc_resolution ) - 1;
  /* Prepare a "half-step" voltage level because when the button number is
	 calculated, the integer gets truncated, not rounded. */
  const int half_step = (int)( ( (float)adc_limit / (float)number_buttons ) ) >> 1;

  int relative_voltage = analogRead( pin );
  int button_voltage = relative_voltage * number_buttons + half_step;
  int button         = button_voltage >> adc_resolution;
  /* Max voltage means no key pressed. */
  if( button == number_buttons )
  {
    return( -1 );
  }
  return( button );
}



/**
 * Get the currently pressed keypad button, if any.  The function
 * makes use of a state machine to handle debouncing.
 *
 * @return Keypad button, or @a -1 if no button is pressed.
 */
int Analog_Buttons::read_key( )
{
  int key_reported = determine_button( );

  /* We're currently debouncing.  Determine if the debounce period has
     expired. */
  if( keypad_state == KEYPAD_STATE_DEBOUNCING )
  {
    /* Accept the current keypad measurement if the debounce period
       has expired, then prepare to repeat the key and go the IDLE state. */
    if( millis( ) > debounce_timestamp + debounce( ) )
    {
      last_key = key_reported;
      repeating_timestamp = millis( );
      keypad_state = KEYPAD_STATE_IDLE;
      return( key_reported );
    }
  }
  /* Any change while the keypad is either idle or repeating a key
     means that we should debounce. */
  else
  {
    /* Debounce the key if the keypad reports a change. */
    if( last_key != key_reported )
    {
      debounce_timestamp = millis( );
      keypad_state = KEYPAD_STATE_DEBOUNCING;
    }
    /* Otherwise, determine if the key has been held down long enough
       to treat it as repeating. */
    else
    {
      int repeat_time = repeat( );
      if( repeat_time > 0 )
      {
        if( millis( ) > repeating_timestamp + repeat_time )
        {
          repeating_timestamp = millis( );
          return( key_reported );
        }
      }
    }
  }

  return( -1 );
}
