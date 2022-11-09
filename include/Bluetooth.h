#pragma once

using BluetoothAction = void(*)(float);

class Bluetooth {
	static const unsigned MAX_COMMAND_COUNT = 20;
	static const unsigned MAX_COMMAND_LENGTH = 20;
	unsigned command_count = 0;
	const char* commands[MAX_COMMAND_COUNT];
	BluetoothAction actions[MAX_COMMAND_COUNT];
	char command_buffer[MAX_COMMAND_LENGTH];
	char number_buffer[MAX_COMMAND_LENGTH];
public:
	Bluetooth() {}

	void add_command(const char* name, BluetoothAction action);
	// Default command for assigning variable;
	void poll();
};