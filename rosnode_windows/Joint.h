#ifndef JOINT_H_
#define JOINT_H_

class Joint {
	public:
		Joint() = delete; // Want custom constructor only
		Joint(char *newName, const int newOffset);
		int get_offset(); // Encapsulation
		int get_size(); // Returns size of object in bytes
		bool exists(char *queryName); // To avoid dupes
	private:
		char *name; // To avoid dupes
		int offset; // Where in shared memory
		int size; // (byte)4 + name length, useful for next offset in a list
};

#endif // JOINT_H
