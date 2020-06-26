#include <stdio.h>
#include <atmel_start.h>
#include <driver_init.h>
#include "geoduck.h"

extern uint32_t debounceTimer;		// free running timer
extern char textline[];

typedef struct {
	uint32_t whichButton;							// port pin
	bool buttonState;									// the official state of the button after debouncing
	bool lastButtonState;							// the previous raw eading from the input pin
	unsigned long lastDebounceTime;		// the last time the output pin was toggled
} buttonUnionType;

// reads and debounces a momentary switch
// returns true if switch has changed
// also returns the current state as a variable
bool readswitch(uint8_t buttonPin, bool *current_state) {
	static bool initialized = false;
	static buttonUnionType buttonUnion[NUM_BUTTONS];
	const unsigned long debounceDelay = 50;    // the debounce time in ms; increase if the output flickers
	bool reading;
	bool changed = false;

	if (!initialized) {
		// initialize
		buttonUnion[LCD_BUTTON_L].whichButton = UI_BUTTON1;
		buttonUnion[LCD_BUTTON_M].whichButton = UI_BUTTON2;
		buttonUnion[LCD_BUTTON_R].whichButton = UI_BUTTON3;
		buttonUnion[GO_BUTTON].whichButton = BUTTON;

		for (int i=0; i<NUM_BUTTONS; i++) {
			buttonUnion[i].buttonState = true;
			buttonUnion[i].lastButtonState = true;
			buttonUnion[i].lastDebounceTime = 0;
		} // for

		initialized = true;
	}

	// save the official state right now
	*current_state = buttonUnion[buttonPin].buttonState;

	// read the state of the switch
	reading = gpio_get_pin_level(buttonUnion[buttonPin].whichButton);

#if VERBOSE
	if (false == reading) {
		sprintf(textline, "pressed %d\r\n", buttonPin);
		USB_PRINT(textline);
	}
#endif

	// check to see if you just pressed the button
	// (i.e. the input went from LOW to HIGH), and you've waited long enough
	// since the last press to ignore any noise:

	// If the switch changed, due to noise or pressing:
	if (reading != buttonUnion[buttonPin].lastButtonState) {
		// reset the debouncing timer
		buttonUnion[buttonPin].lastDebounceTime = debounceTimer;
	}

	if ((debounceTimer - buttonUnion[buttonPin].lastDebounceTime) > debounceDelay) {
		// whatever the reading is at, it's been there for longer than the debounce
		// delay, so take it as the new official state
		buttonUnion[buttonPin].lastDebounceTime = debounceTimer;
		buttonUnion[buttonPin].buttonState = reading;
	}

	// compare saved buttonState with new official buttonState
	changed = *current_state != buttonUnion[buttonPin].buttonState;

	// assign the new official buttonState
	*current_state = buttonUnion[buttonPin].buttonState;

	// save the reading. Next time through the loop, it'll be the lastButtonState
	buttonUnion[buttonPin].lastButtonState = reading;

	return (changed);
}

