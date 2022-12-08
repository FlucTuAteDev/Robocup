#include "variables.h"
#include <Bluetooth.h>
#include <Arduino.h>

void Bluetooth::setup_commands() {
	using namespace variables;
	add_command("motormax", [](float num) { MotorController::max_motor_speed = num; });
	add_command("motorch", [](float num) { MotorController::max_change = num; });
	add_command("i", [](float num) { i = num; });
	add_command("p", [](float num) { p = num; });
	add_command("d", [](float num) { d = num; });
	add_command("si", [](float num) { i_speed = num; });
	add_command("sp", [](float num) { p_speed = num; });
	add_command("sd", [](float num) { d_speed = num; });
	add_command("start", [](float num) { mc.start(); sampling = false; });
	add_command("stop", [](float num) { mc.stop(); reset(); });
	add_command("reset", [](float num) { reset(); });
	add_command("status", [](float num) {
		Serial.println((String)"p: " + p + "; i: " + i + "; d: " + d);
		Serial.println((String)"Speed p: " + p_speed + "; i: " + i_speed + "; d: " + d_speed);
		Serial.println((String)"Balance angle: " + balance_angle);
		Serial.println((String)"Pulse left: " + cumpulseleft + "; right: " + cumpulseright);
		Serial.println((String)"Positions: " + positions);
		// Serial.println((String)"Speeds filter: " + speeds_filter + "; filterold: " + speeds_filterold);
		Serial.println((String)"PD_pwm: " + PD_pwm); 
		Serial.println((String)"Current phase index: " + curr_phase_index);
	});
	add_command("sample", [](float num) {
		mc.stop();
		sampling = true;
		// Start sampling around the angle at which the command is issued
		balance_angle = tilt_angle; 
	});
	add_command("sample_stop", [](float num) { sampling = false; });
	add_command("pos", [](float num) { positions += num; });
	add_command("posc", [](float num) { pos_constrain = num; });
	add_command("pulser", [](float num) { cumpulseright += num; });
}

void Bluetooth::add_command(const char *name, BluetoothAction action)
{
	if (command_count == MAX_COMMAND_COUNT)
		return;

	if (strlen(name) > MAX_COMMAND_LENGTH)
		return;

	commands[command_count] = name;
	actions[command_count++] = action;
}

void Bluetooth::poll()
{
	if (!Serial.available())
		return;

	// Wait for the entire command to get through (9600 bps baud rate => 1.2 char / ms)
	delay(20);
	double number = 0;
	for (size_t buffer_index = 0; Serial.available(); buffer_index++)
	{
		char current = Serial.read();
		if (current == ' ')
		{
			command_buffer[buffer_index] = '\0';

			// Read number
			for (size_t number_index = 0; Serial.available(); number_index++)
			{
				current = Serial.read();
				if (current == '\n')
				{
					number_buffer[number_index] = '\0';
					break;
				}

				number_buffer[number_index] = current;
			}

			number = atof(number_buffer);
			break;
		}

		if (current == '\n')
		{
			command_buffer[buffer_index] = '\0';
			break;
		}

		if (buffer_index == MAX_COMMAND_LENGTH) return;
		
		command_buffer[buffer_index] = current;
	}
	
	// Search for action related to the command
	BluetoothAction action = nullptr;
	for (size_t i = 0; i < command_count; i++)
	{
		if (!strcmp(command_buffer, commands[i]))
		{
			action = actions[i];
		}
	}
	if (!action)
	{
		Serial.println("No such command!");
		return;
	}
	
	action(number);
	Serial.println("OK");
}