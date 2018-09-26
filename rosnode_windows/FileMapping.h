#ifndef FILE_MAPPING_H_
#define FILE_MAPPING_H_

//#define DEBUG

#include <windows.h>
#include <stdint.h>
#include <vector>

class FileMapping {
	public:
		HANDLE handle;
		LPCTSTR buffer; // Shared memory
		FileMapping() = delete; // Don't use default constructor
		FileMapping(const char *newName, int size); // newName must match other end
		~FileMapping(); // Unmap and close memory
		int getBufferSize(); // Accessor function
		int open_fileMap(); // Open and map memory
		void write_data(uint8_t datum, int offset); // Encapsulates storing data
		void write_data(float datum, int offset); // Encapsulates storing data
		void write_data(const char* string, int offset); // Encapsulates storing data
		uint8_t read_data(int offset); // Encapsulates reading data
		float read_data_float(int offset); // Encapsulates reading data
		void commit_data(); // Writes buffer to shared memory
		void commit_data(int size); // Writes section of buffer to shared memory
		uint8_t *get_data();
	  #ifdef DEBUG
		void debug_memory(); // Prints contents of data
	  #endif // DEBUG
	private:
		LPCSTR name; // Filled by newName
		uint8_t *data; // Can be read or written
		int bufferSize;
	  #ifdef DEBUG
		uint8_t *debug;
	  #endif // DEBUG
};

#endif // FILE_MAPPING_H_
