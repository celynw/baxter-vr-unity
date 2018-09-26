#include "Joint.h"
#include <string>

#include <iostream>
#include <windows.h>
#include <string>
#include <tchar.h>

Joint::Joint(char *newName, const int newOffset) {
	offset = newOffset;
	name = _strdup(newName);
	// Joints are double, we are using floats
	size = strlen(name) + sizeof(float);
}

// Encapsulation
int Joint::get_offset() {
	return offset;
}

// (byte)4 + name length, useful for next offset in a list
int Joint::get_size() {
	return size;
}

// To avoid dupes
bool Joint::exists(char *queryName) {
	return (!strcmp(queryName, name));
}
