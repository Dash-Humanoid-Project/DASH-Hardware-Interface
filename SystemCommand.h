#pragma once

#include <vector>
#include <cstring>
#include <iostream>

// SystemCommand structure for Position Control
struct SystemCommand {
    float position;		// desired position
    float velocity_ff;  // velocity feedforward

	// Serialize SystemCommand into a vector of uint8_t
	std::vector<uint8_t> serialize()
    {
		std::vector<uint8_t> data(sizeof(SystemCommand));

		// Copy the position and velocity_ff into the buffer
		std::memcpy(data.data(), &position, sizeof(position));
		std::memcpy(data.data() + sizeof(position), &velocity_ff, sizeof(velocity_ff));

		return data;
	}

	// Deserialize froma buffer of uint8_t to populate the SystemCommand fields
	static SystemCommand deserialize(const std::vector<uint8_t>& buffer)
    {
		SystemCommand cmd;

		// Check if the buffer is large enough to hold the data
		if (buffer.size() >= sizeof(SystemCommand)) {
				// Copy the position and velocity_ff from the buffer
				std::memcpy(&cmd.position, buffer.data(), sizeof(cmd.position));
				std::memcpy(&cmd.velocity_ff, buffer.data() + sizeof(cmd.position), sizeof(cmd.velocity_ff));
		} else {
				std::cerr << "Error: Buffer size is too small to deserialize into SystemCommand." << std::endl;
		}

		return cmd;
	}

};
