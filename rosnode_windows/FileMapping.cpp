#include "FileMapping.h"
#include <iostream>
#include <windows.h>
#include <string>
#include <tchar.h>

// newName must match other end
FileMapping::FileMapping(const char *newName, int size) {
	std::cout << "fileMapping opened: " << newName << std::endl;
	name = (LPCSTR)newName;
	bufferSize = size;
	data = new uint8_t[bufferSize];
#ifdef DEBUG
	debug = new uint8_t[bufferSize];
#endif // DEBUG
}

// Unmap and close memory
FileMapping::~FileMapping() {
	UnmapViewOfFile(buffer);
	CloseHandle(handle);
	delete data;
#ifdef DEBUG
	delete debug;
#endif // DEBUG
}

// Accessor function
int FileMapping::getBufferSize() {
	return bufferSize;
}

// Open and map memory
int FileMapping::open_fileMap() {
	// use paging file
	// default security
	// read/write access
	// maximum object size (high-order DWORD)
	// maximum object size (low-order DWORD)
	// name of mapping object
	handle = CreateFileMappingA(INVALID_HANDLE_VALUE, NULL, PAGE_READWRITE, 0, bufferSize, name);
	if (handle == NULL) {
		int err = GetLastError();
		std::cout << "Could not create file mapping object \"" << name << "\"" << std::endl;
		std::cout << "(" << err << ")" << std::endl;
		if (err == 5)
			std::cout << "Maybe you aren't Administrator" << std::endl;
		return err;
	}
	// handle to map object, read/write permission
	buffer = (LPTSTR)MapViewOfFile(handle, FILE_MAP_ALL_ACCESS, 0, 0, bufferSize);
	if (buffer == NULL) {
		std::cout << "Could not map view of file: " << GetLastError() << std::endl;
		CloseHandle(handle);
		return 1;
	}
	else {
		std::cout << "File mapped successfully" << std::endl;
	}

	ZeroMemory((PVOID)buffer, 1); // Not sure if required
	return 0;
}

// Encapsulates storing data
void FileMapping::write_data(uint8_t datum, int offset) {
	data[offset] = datum;
}

// Encapsulates storing data
void FileMapping::write_data(float datum, int offset) {
	///std::cout << "  float... " << offset << std::endl;
	///std::cout << offset << ": " << datum << std::endl;

	uint8_t* datumBytes = reinterpret_cast<uint8_t*>(&datum);
	for (int o(0); o < sizeof(float); o++)
		write_data(datumBytes[o], offset + o);
}

// Encapsulates storing data
void FileMapping::write_data(const char* string, int offset) {
	///std::cout << "  char... " << offset << std::endl;
	for (uint8_t c(0); c < strlen(string); c++)
		write_data((uint8_t)string[c], offset + c);
}

// Encapsulates reading data
uint8_t FileMapping::read_data(int offset) {
	CopyMemory(data, buffer+(offset/2), 1); //CHECKME?
	return data[0];
	//return (int)*(buffer + offset);
}

// Encapsulates reading data
float FileMapping::read_data_float(int offset) {
	float result;
	// BUG?!
	CopyMemory((PVOID)data, buffer+(offset/2), sizeof(float));
	std::copy(reinterpret_cast<const char*>(&data[0]),
		reinterpret_cast<const char*>(&data[sizeof(float)]),
		reinterpret_cast<char*>(&result));
	return result;
}

// Writes buffer to shared memory
void FileMapping::commit_data() {
	CopyMemory((PVOID)buffer, data, bufferSize);
}

// Writes section of buffer to shared memory
void FileMapping::commit_data(int size) {
	CopyMemory((PVOID)buffer, data, size);
}

uint8_t *FileMapping::get_data() {
	CopyMemory((PVOID)buffer, data, bufferSize);
	return data;
}


// Prints contents of data
#ifdef DEBUG
void FileMapping::debug_memory() {
	CopyMemory(debug, data, bufferSize);
	std::cout << "Memory:" << std::endl;
	for (int i(0); i < bufferSize; i++)
		std::cout << " " << i << ": " << (int)debug[i] << std::endl;
	std::cout << std::endl;
}
#endif // DEBUG
