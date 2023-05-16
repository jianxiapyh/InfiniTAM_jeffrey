// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Scene/ITMVoxelBlockHash.h"
#include "../../../ORUtils/Image.h"

#include <stdlib.h>

namespace ITMLib
{
	class ITMMesh
	{
	public:
		struct Triangle { Vector3f p0, p1, p2, clr0, clr1, clr2; };

		MemoryDeviceType memoryType;

		uint noTotalTriangles;
		// Number of bricks x voxels/brick
		static const uint noMaxTriangles_default = SDF_LOCAL_BLOCK_NUM * SDF_BLOCK_SIZE3;
		uint noMaxTriangles;

		ORUtils::MemoryBlock<Triangle> *triangles;

		explicit ITMMesh(MemoryDeviceType memoryType, uint maxTriangles = noMaxTriangles_default)
		{
			this->memoryType = memoryType;
			this->noTotalTriangles = 0;
			this->noMaxTriangles = maxTriangles;

			triangles = new ORUtils::MemoryBlock<Triangle>(noMaxTriangles, memoryType);
		}

		void WriteOBJ(const char *fileName)
		{
			ORUtils::MemoryBlock<Triangle> *cpu_triangles; bool shouldDelete = false;
			if (memoryType == MEMORYDEVICE_CUDA)
			{
				cpu_triangles = new ORUtils::MemoryBlock<Triangle>(noMaxTriangles, MEMORYDEVICE_CPU);
				cpu_triangles->SetFrom(triangles, ORUtils::MemoryBlock<Triangle>::CUDA_TO_CPU);
				shouldDelete = true;
			}
			else cpu_triangles = triangles;

			Triangle *triangleArray = cpu_triangles->GetData(MEMORYDEVICE_CPU);

			FILE *f = fopen(fileName, "w+");
			if (f != NULL)
			{
				for (uint i = 0; i < noTotalTriangles; i++)
				{
                    //remove color
                    
                    //fprintf(f, "v %f %f %f\n", triangleArray[i].p0.x, triangleArray[i].p0.y, triangleArray[i].p0.z);
					//fprintf(f, "v %f %f %f\n", triangleArray[i].p1.x, triangleArray[i].p1.y, triangleArray[i].p1.z);
					//fprintf(f, "v %f %f %f\n", triangleArray[i].p2.x, triangleArray[i].p2.y, triangleArray[i].p2.z);
                    
                    //convert color from float into uint8
                    //https://github.com/google/draco/issues/105
                    uint8_t int_color0[3];
                    uint8_t int_color1[3];
                    uint8_t int_color2[3];
                    int_color0[0] = static_cast<int>(triangleArray[i].clr0.r); int_color0[1] = static_cast<int>(triangleArray[i].clr0.g); int_color0[2] = static_cast<int>(triangleArray[i].clr0.b);
                    //fprintf(f, "test: %u %u %u\n", int_color0[0], int_color0[1], int_color0[2]);
                    fprintf(f, "v %f %f %f %u %u %u\n", triangleArray[i].p0.x, triangleArray[i].p0.y, triangleArray[i].p0.z, int_color0[0], int_color0[1], int_color0[2]);
                    
                    int_color1[0] = static_cast<int>(triangleArray[i].clr1.r); int_color1[1] = static_cast<int>(triangleArray[i].clr1.g); int_color1[2] = static_cast<int>(triangleArray[i].clr1.b);
					fprintf(f, "v %f %f %f %u %u %u\n", triangleArray[i].p1.x, triangleArray[i].p1.y, triangleArray[i].p1.z, int_color1[0], int_color1[1], int_color1[2]);
                    
                    int_color2[0] = static_cast<int>(triangleArray[i].clr2.r); int_color2[1] = static_cast<int>(triangleArray[i].clr2.g); int_color2[2] = static_cast<int>(triangleArray[i].clr2.b);
					fprintf(f, "v %f %f %f %u %u %u\n", triangleArray[i].p2.x, triangleArray[i].p2.y, triangleArray[i].p2.z, int_color2[0], int_color2[1], int_color2[2]);
                    
                    
                    //original
                    //fprintf(f, "v %f %f %f %f %f %f\n", triangleArray[i].p0.x, triangleArray[i].p0.y, triangleArray[i].p0.z, triangleArray[i].clr0.r, triangleArray[i].clr0.g, triangleArray[i].clr0.b);
					//fprintf(f, "v %f %f %f %f %f %f\n", triangleArray[i].p1.x, triangleArray[i].p1.y, triangleArray[i].p1.z, triangleArray[i].clr1.r, triangleArray[i].clr1.g, triangleArray[i].clr1.b);
					//fprintf(f, "v %f %f %f %f %f %f\n", triangleArray[i].p2.x, triangleArray[i].p2.y, triangleArray[i].p2.z, triangleArray[i].clr2.r, triangleArray[i].clr2.g, triangleArray[i].clr2.b);
				}

				for (uint i = 0; i<noTotalTriangles; i++) fprintf(f, "f %d %d %d\n", i * 3 + 2 + 1, i * 3 + 1 + 1, i * 3 + 0 + 1);
				fclose(f);
			}

			if (shouldDelete) delete cpu_triangles;
		}

		void WriteSTL(const char *fileName)
		{
			ORUtils::MemoryBlock<Triangle> *cpu_triangles; bool shoulDelete = false;
			if (memoryType == MEMORYDEVICE_CUDA)
			{
				cpu_triangles = new ORUtils::MemoryBlock<Triangle>(noMaxTriangles, MEMORYDEVICE_CPU);
				cpu_triangles->SetFrom(triangles, ORUtils::MemoryBlock<Triangle>::CUDA_TO_CPU);
				shoulDelete = true;
			}
			else cpu_triangles = triangles;

			Triangle *triangleArray = cpu_triangles->GetData(MEMORYDEVICE_CPU);

			FILE *f = fopen(fileName, "wb+");

			if (f != NULL) {
				for (int i = 0; i < 80; i++) fwrite(" ", sizeof(char), 1, f);

				fwrite(&noTotalTriangles, sizeof(int), 1, f);

				float zero = 0.0f; short attribute = 0;
				for (uint i = 0; i < noTotalTriangles; i++)
				{
					fwrite(&zero, sizeof(float), 1, f); fwrite(&zero, sizeof(float), 1, f); fwrite(&zero, sizeof(float), 1, f);

					fwrite(&triangleArray[i].p2.x, sizeof(float), 1, f); 
					fwrite(&triangleArray[i].p2.y, sizeof(float), 1, f); 
					fwrite(&triangleArray[i].p2.z, sizeof(float), 1, f);

					fwrite(&triangleArray[i].p1.x, sizeof(float), 1, f); 
					fwrite(&triangleArray[i].p1.y, sizeof(float), 1, f); 
					fwrite(&triangleArray[i].p1.z, sizeof(float), 1, f);

					fwrite(&triangleArray[i].p0.x, sizeof(float), 1, f);
					fwrite(&triangleArray[i].p0.y, sizeof(float), 1, f);
					fwrite(&triangleArray[i].p0.z, sizeof(float), 1, f);

					fwrite(&attribute, sizeof(short), 1, f);

					//fprintf(f, "v %f %f %f\n", triangleArray[i].p0.x, triangleArray[i].p0.y, triangleArray[i].p0.z);
					//fprintf(f, "v %f %f %f\n", triangleArray[i].p1.x, triangleArray[i].p1.y, triangleArray[i].p1.z);
					//fprintf(f, "v %f %f %f\n", triangleArray[i].p2.x, triangleArray[i].p2.y, triangleArray[i].p2.z);
				}

				//for (uint i = 0; i<noTotalTriangles; i++) fprintf(f, "f %d %d %d\n", i * 3 + 2 + 1, i * 3 + 1 + 1, i * 3 + 0 + 1);
				fclose(f);
			}

			if (shoulDelete) delete cpu_triangles;
		}

		~ITMMesh()
		{
			delete triangles;
		}

		// Suppress the default copy constructor and assignment operator
		ITMMesh(const ITMMesh&);
		ITMMesh& operator=(const ITMMesh&);
	};
}
