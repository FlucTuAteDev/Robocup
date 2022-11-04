#include <bluetooth.h>

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
	delay(10);
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

	Serial.println(command_buffer);
	Serial.println(number_buffer);
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
}