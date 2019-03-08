//
// Simple Wavefront .obj parser in C
// Anton Gerdelan 22 Dec 2014
// antongerdelan.net
//
#ifndef OBJ_PARSER_H_D15D9A28_4191_11E9_9ADD_BB963C4C117C
#define OBJ_PARSER_H_D15D9A28_4191_11E9_9ADD_BB963C4C117C

#include <stdbool.h>

bool load_obj_file (
	const char* file_name,
	float** points,
	float** tex_coords,
	float** normals,
	int* point_count
);

#endif // OBJ_PARSER_H_D15D9A28_4191_11E9_9ADD_BB963C4C117C
