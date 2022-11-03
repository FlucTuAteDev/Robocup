#pragma once
#include <Arduino.h>

using BluetoothAction = void(*)(float);

class Bluetooth {
	static const unsigned MAX_COUNT = 20;
	unsigned count = 0;
	const char* commands[MAX_COUNT];
	BluetoothAction actions[MAX_COUNT];
	char command_buffer[20];
	char number_buffer[20];
public:
	Bluetooth() {}

	void add_command(const char* name, BluetoothAction action);
	void poll();
};