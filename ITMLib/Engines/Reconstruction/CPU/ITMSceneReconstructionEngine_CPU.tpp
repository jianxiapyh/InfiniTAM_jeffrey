// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#include "ITMSceneReconstructionEngine_CPU.h"

#include "../Shared/ITMSceneReconstructionEngine_Shared.h"
#include "../../../Objects/RenderStates/ITMRenderState_VH.h"
#include "../../../../ORUtils/Trace.h"

#include <memory>
#include <sstream>

using namespace ITMLib;

// Tracing data
bool ORUtils::Trace::enabled_ = false;
static uint32_t count{0u};
static constexpr int start_frame = 30;
static constexpr int end_frame = 34;

static const std::string TRACE_FILE_CONFIG{"it_"};
static const std::string TRACE_FILE_FOLDER{"./"};
static const std::string TRACE_FILE_PREFIX{
	TRACE_FILE_FOLDER + TRACE_FILE_CONFIG + std::to_string(start_frame) + "_" + std::to_string(end_frame) + "_"};

static const std::string TRACE_FILE_VC_PIXEL{TRACE_FILE_PREFIX + "vc_pixels.gz"};
static const std::string TRACE_FILE_DF_BRICK{TRACE_FILE_PREFIX + "df_bricks.gz"};
static const std::string TRACE_FILE_DF_VOXEL{TRACE_FILE_PREFIX + "df_voxels.gz"};

/// Same as the original but also collects a functional trace
void buildHashAllocAndVisibleTypePPTrace(DEVICEPTR(uchar) *entriesAllocType, DEVICEPTR(uchar) *entriesVisibleType, int x, int y,
	DEVICEPTR(Vector4s) *blockCoords, const CONSTPTR(float) *depth, Matrix4f invM_d, Vector4f projParams_d, float mu, Vector2i imgSize,
	float oneOverVoxelSize, const CONSTPTR(ITMHashEntry) *hashTable, float viewFrustum_min, float viewFrustum_max,
	const int pixelId, std::ostream& os)
{
	float depth_measure; unsigned int hashIdx; int noSteps;
	Vector4f pt_camera_f; Vector3f point_e, point, direction; Vector3s blockPos;

	// Inst 0 begin
	depth_measure = depth[x + y * imgSize.x];
	ORUtils::Trace::writeMemoryInstruction(os, pixelId, 0, "load", "global", "depth", std::addressof(depth[x + y * imgSize.x]));
	// Inst 0 end; global memory load latency cycles

	// Exit if depth reading is invalid or outside viewing frustum
	if (depth_measure <= 0 || (depth_measure - mu) < 0 || (depth_measure - mu) < viewFrustum_min || (depth_measure + mu) > viewFrustum_max)
		return;

	// Inst 1 begin
	pt_camera_f.z = depth_measure;
	pt_camera_f.x = pt_camera_f.z * ((float(x) - projParams_d.z) * projParams_d.x);
	pt_camera_f.y = pt_camera_f.z * ((float(y) - projParams_d.w) * projParams_d.y);

	float norm = sqrt(pt_camera_f.x * pt_camera_f.x + pt_camera_f.y * pt_camera_f.y + pt_camera_f.z * pt_camera_f.z);

	Vector4f pt_buff;

	pt_buff = pt_camera_f * (1.0f - mu / norm); pt_buff.w = 1.0f;
	point = TO_VECTOR3(invM_d * pt_buff) * oneOverVoxelSize;

	pt_buff = pt_camera_f * (1.0f + mu / norm); pt_buff.w = 1.0f;
	point_e = TO_VECTOR3(invM_d * pt_buff) * oneOverVoxelSize;

	direction = point_e - point;
	norm = sqrt(direction.x * direction.x + direction.y * direction.y + direction.z * direction.z);
	noSteps = (int)ceil(2.0f*norm);

	direction /= (float)(noSteps - 1);
	ORUtils::Trace::writeComputeInstruction(os, pixelId, 1, 85);
	// Inst 1 end; 15*5 (3 divs and 2 sqrts) + 10 = 85 cycles

	//add neighbouring blocks
	for (int i = 0; i < noSteps; i++)
	{
		// Inst 2 begin
		blockPos = TO_SHORT_FLOOR3(point);

		//compute index in hash table
		hashIdx = hashIndex(blockPos);
		ORUtils::Trace::writeComputeInstruction(os, pixelId, 2, 1);
		// Inst 2 end; 1 cycle

		//check if hash table contains entry
		bool isFound = false;

		// Inst 3, 4, 5 begin
		ITMHashEntry hashEntry = hashTable[hashIdx];
		ORUtils::Trace::writeMemoryInstruction(os, pixelId, 3, "load", "global", "hash", std::addressof(hashTable[hashIdx].pos));
		ORUtils::Trace::writeMemoryInstruction(os, pixelId, 4, "load", "global", "hash", std::addressof(hashTable[hashIdx].offset));
		ORUtils::Trace::writeMemoryInstruction(os, pixelId, 5, "load", "global", "hash", std::addressof(hashTable[hashIdx].ptr));
		// Inst 3, 4, 5 end; global memory load latency cycles

		// Inst 6 begin
		if (IS_EQUAL3(hashEntry.pos, blockPos) && hashEntry.ptr >= -1)
		{
			//entry has been streamed out but is visible or in memory and visible
			entriesVisibleType[hashIdx] = (hashEntry.ptr == -1) ? 2 : 1;

			isFound = true;
		}
		ORUtils::Trace::writeComputeInstruction(os, pixelId, 6, 1);
		// Inst 6 end; 1 cycle

		if (!isFound)
		{
			bool isExcess = false;
			if (hashEntry.ptr >= -1) //seach excess list only if there is no room in ordered part
			{
				while (hashEntry.offset >= 1)
				{
					// Inst 7, 8, 9 begin
					hashIdx = SDF_BUCKET_NUM + hashEntry.offset - 1;
					hashEntry = hashTable[hashIdx];
					ORUtils::Trace::writeMemoryInstruction(os, pixelId, 7, "load", "global", "hash", std::addressof(hashTable[hashIdx].pos));
					ORUtils::Trace::writeMemoryInstruction(os, pixelId, 8, "load", "global", "hash", std::addressof(hashTable[hashIdx].offset));
					ORUtils::Trace::writeMemoryInstruction(os, pixelId, 9, "load", "global", "hash", std::addressof(hashTable[hashIdx].ptr));
					// Inst 7, 8, 9 end; global memory load latency cycles

					// Inst 10 begin
					if (IS_EQUAL3(hashEntry.pos, blockPos) && hashEntry.ptr >= -1)
					{
						//entry has been streamed out but is visible or in memory and visible
						entriesVisibleType[hashIdx] = (hashEntry.ptr == -1) ? 2 : 1;

						isFound = true;
						ORUtils::Trace::writeComputeInstruction(os, pixelId, 10, 1);
						// Inst 10 end; 1 cycle

						break;
					}
				}

				isExcess = true;
			}

			// Inst 11 begin
			if (!isFound) //still not found
			{
				entriesAllocType[hashIdx] = isExcess ? 2 : 1; //needs allocation
				if (!isExcess) entriesVisibleType[hashIdx] = 1; //new entry is visible

				blockCoords[hashIdx] = Vector4s(blockPos.x, blockPos.y, blockPos.z, 1);
			}
			ORUtils::Trace::writeComputeInstruction(os, pixelId, 11, 1);
			// Inst 11 end; 1 cycle
		}

		point += direction;
	}
}

/// Same as the original but also collects a functional trace
template<class TVoxel>
float computeUpdatedVoxelDepthInfoTrace(DEVICEPTR(TVoxel) &voxel, const THREADPTR(Vector4f) & pt_model, const CONSTPTR(Matrix4f) & M_d,
	const CONSTPTR(Vector4f) & projParams_d, float mu, int maxW, const CONSTPTR(float) *depth, const CONSTPTR(Vector2i) & imgSize,
	const int brickId, const int voxelId, std::ostream& os)
{
	Vector4f pt_camera; Vector2f pt_image;
	float depth_measure, eta, oldF, newF;
	int oldW, newW;

	// Transform point from world coordinates to camera coordinates
	// Inst 0 begin
	pt_camera = M_d * pt_model;
	ORUtils::Trace::writeComputeInstruction(os, voxelId, 0, 1);
	// Inst 0 end; 1 cycle

	// Exit if point is behind the camera
	if (pt_camera.z <= 0)
		return -1;

	// Project point into image
	// Inst 1 begin
	pt_image.x = projParams_d.x * pt_camera.x / pt_camera.z + projParams_d.z;
	pt_image.y = projParams_d.y * pt_camera.y / pt_camera.z + projParams_d.w;
	ORUtils::Trace::writeComputeInstruction(os, voxelId, 1, 17);
	// Inst 1 end; 1   + 15  + 1   = 17 cycles
	//             mul + div + add = total

	// Exit if point is outside the image
	if ((pt_image.x < 1) || (pt_image.x > imgSize.x - 2) || (pt_image.y < 1) || (pt_image.y > imgSize.y - 2))
		return -1;

	// Get measured depth from image
	// Inst 2 begin
	const int depth_idx = (int)(pt_image.x + 0.5f) + (int)(pt_image.y + 0.5f) * imgSize.x;
	depth_measure = depth[depth_idx];
	ORUtils::Trace::writeMemoryInstruction(os, voxelId, 2, "load", "global", "depth", std::addressof(depth[depth_idx]));
	// Inst 2 end; global memory load latency cycles

	// Exit if depth reading is invalid
	if (depth_measure <= 0.0f)
		return -1;

	// Check whether voxel needs updating
	// Inst 3 begin
	eta = depth_measure - pt_camera.z;
	ORUtils::Trace::writeComputeInstruction(os, voxelId, 3, 1);
	// Inst 3 end; 1 cycle

	// Exit if voxel is outside truncation band
	if (eta < -mu)
		return eta;

	// Compute updated SDF value and reliability

	// Inst 4 begin
	oldF = TVoxel::valueToFloat(voxel.sdf);
	ORUtils::Trace::writeMemoryInstruction(os, voxelId, 4, "load", "global", "brick", std::addressof(voxel.sdf));
	// Inst 4 end; global memory load latency cycles

	// Inst 5 begin
	oldW = voxel.w_depth;

	// Since voxel.w_depth falls into the same word as voxel.sdf, we don't need to record its access. However, we do need to
	// record the access to voxel.clr. We do so here because the actual access happens in another function.
	ORUtils::Trace::writeMemoryInstruction(os, voxelId, 5, "load", "global", "brick", std::addressof(voxel.clr));
	// Inst 5 end; global memory load latency cycles

	// Inst 6 begin
	newF = MIN(1.0f, eta / mu);
	newW = 1;

	newF = oldW * oldF + newW * newF;
	newW = oldW + newW;
	newF /= newW;
	newW = MIN(newW, maxW);
	ORUtils::Trace::writeComputeInstruction(os, voxelId, 6, 36);
	// Inst 6 end; 15  + 1    + 15  + 5         = 36 cycles
	//             div + madd + div + for color = total

	// Write back

	// Inst 7 begin
	voxel.sdf = TVoxel::floatToValue(newF);
	ORUtils::Trace::writeMemoryInstruction(os, voxelId, 7, "store", "global", "brick", std::addressof(voxel.sdf));
	// Inst 7 end; global memory store latency cycles

	// Inst 8 begin
	voxel.w_depth = newW;

	// Since voxel.w_depth falls into the same word as voxel.sdf, we don't need to record its access. However, we do need to
	// record the access to voxel.clr. We do so here because the actual access happens in another function.
	ORUtils::Trace::writeMemoryInstruction(os, voxelId, 8, "store", "global", "brick", std::addressof(voxel.clr));
	// Inst 8 end; global memory store latency cycles

	return eta;
}

template<class TVoxel>
ITMSceneReconstructionEngine_CPU<TVoxel,ITMVoxelBlockHash>::ITMSceneReconstructionEngine_CPU(void)
{
	int noTotalEntries = ITMVoxelBlockHash::noTotalEntries;
	entriesAllocType = new ORUtils::MemoryBlock<unsigned char>(noTotalEntries, MEMORYDEVICE_CPU);
	blockCoords = new ORUtils::MemoryBlock<Vector4s>(noTotalEntries, MEMORYDEVICE_CPU);

	// Clear trace files
	ORUtils::Trace::clearFile(TRACE_FILE_VC_PIXEL);
	ORUtils::Trace::clearFile(TRACE_FILE_DF_BRICK);
	ORUtils::Trace::clearFile(TRACE_FILE_DF_VOXEL);
}

template<class TVoxel>
ITMSceneReconstructionEngine_CPU<TVoxel,ITMVoxelBlockHash>::~ITMSceneReconstructionEngine_CPU(void)
{
	delete entriesAllocType;
	delete blockCoords;
}

template<class TVoxel>
void ITMSceneReconstructionEngine_CPU<TVoxel,ITMVoxelBlockHash>::ResetScene(ITMScene<TVoxel, ITMVoxelBlockHash> *scene)
{
	int numBlocks = scene->index.getNumAllocatedVoxelBlocks();
	int blockSize = scene->index.getVoxelBlockSize();

	TVoxel *voxelBlocks_ptr = scene->localVBA.GetVoxelBlocks();
	for (int i = 0; i < numBlocks * blockSize; ++i) voxelBlocks_ptr[i] = TVoxel();
	int *vbaAllocationList_ptr = scene->localVBA.GetAllocationList();
	for (int i = 0; i < numBlocks; ++i) vbaAllocationList_ptr[i] = i;
	scene->localVBA.lastFreeBlockId = numBlocks - 1;

	ITMHashEntry tmpEntry;
	memset(&tmpEntry, 0, sizeof(ITMHashEntry));
	tmpEntry.ptr = -2;
	ITMHashEntry *hashEntry_ptr = scene->index.GetEntries();
	for (int i = 0; i < scene->index.noTotalEntries; ++i) hashEntry_ptr[i] = tmpEntry;
	int *excessList_ptr = scene->index.GetExcessAllocationList();
	for (int i = 0; i < SDF_EXCESS_LIST_SIZE; ++i) excessList_ptr[i] = i;

	scene->index.SetLastFreeExcessListId(SDF_EXCESS_LIST_SIZE - 1);
}

template<class TVoxel>
void ITMSceneReconstructionEngine_CPU<TVoxel, ITMVoxelBlockHash>::IntegrateIntoScene(ITMScene<TVoxel, ITMVoxelBlockHash> *scene, const ITMView *view,
	const ITMTrackingState *trackingState, const ITMRenderState *renderState)
{
	Vector2i rgbImgSize = view->rgb->noDims;
	Vector2i depthImgSize = view->depth->noDims;
	float voxelSize = scene->sceneParams->voxelSize;

	Matrix4f M_d, M_rgb;
	Vector4f projParams_d, projParams_rgb;

	ITMRenderState_VH *renderState_vh = (ITMRenderState_VH*)renderState;

	M_d = trackingState->pose_d->GetM();
	if (TVoxel::hasColorInformation) M_rgb = view->calib.trafo_rgb_to_depth.calib_inv * M_d;

	projParams_d = view->calib.intrinsics_d.projectionParamsSimple.all;
	projParams_rgb = view->calib.intrinsics_rgb.projectionParamsSimple.all;

	float mu = scene->sceneParams->mu; int maxW = scene->sceneParams->maxW;

	float *depth = view->depth->GetData(MEMORYDEVICE_CPU);
	float *confidence = view->depthConfidence->GetData(MEMORYDEVICE_CPU);
	Vector4u *rgb = view->rgb->GetData(MEMORYDEVICE_CPU);
	TVoxel *localVBA = scene->localVBA.GetVoxelBlocks();
	ITMHashEntry *hashTable = scene->index.GetEntries();

	int *visibleEntryIds = renderState_vh->GetVisibleEntryIDs();
	int noVisibleEntries = renderState_vh->noVisibleEntries;

	bool stopIntegratingAtMaxW = scene->sceneParams->stopIntegratingAtMaxW;
	//bool approximateIntegration = !trackingState->requiresFullRendering;

	// Prevent segfault. TODO: obtain actual value
	scene->averageConfidence.push_back(0);

	std::stringstream bss;
	std::stringstream vss;

#ifdef WITH_OPENMP
	#pragma omp parallel for
#endif
	for (int entryId = 0; entryId < noVisibleEntries; entryId++)
	{
		Vector3i globalPos;

		// Inst 0 begin
		const int hashIndex = visibleEntryIds[entryId];
		ORUtils::Trace::writeMemoryInstruction(bss, entryId, 0, "load", "global", "vis", std::addressof(visibleEntryIds[entryId]));
		// Inst 0 end; global memory load latency cycles

		// Inst 1, 2 begin
		const ITMHashEntry &currentHashEntry = hashTable[hashIndex];
		ORUtils::Trace::writeMemoryInstruction(bss, entryId, 1, "load", "global", "hash", std::addressof(hashTable[hashIndex].pos));
		ORUtils::Trace::writeMemoryInstruction(bss, entryId, 2, "load", "global", "hash", std::addressof(hashTable[hashIndex].ptr));
		// Inst 1, 2 end; global memory load latency cycles

		// Inst 3 begin
		if (currentHashEntry.ptr < 0) continue;

		globalPos.x = currentHashEntry.pos.x;
		globalPos.y = currentHashEntry.pos.y;
		globalPos.z = currentHashEntry.pos.z;
		globalPos *= SDF_BLOCK_SIZE;
		ORUtils::Trace::writeComputeInstruction(bss, entryId, 3, 1);
		// Inst 3 end; 1 cycle

		// This is not an instruction because it isn't actually a load. It's just a reference.
		const int vbaIndex = currentHashEntry.ptr * SDF_BLOCK_SIZE3;
		TVoxel *localVoxelBlock = &(localVBA[vbaIndex]);

		for (int z = 0; z < SDF_BLOCK_SIZE; z++) for (int y = 0; y < SDF_BLOCK_SIZE; y++) for (int x = 0; x < SDF_BLOCK_SIZE; x++)
		{
			Vector4f pt_model; int locId;

			locId = x + y * SDF_BLOCK_SIZE + z * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE;

			if (stopIntegratingAtMaxW) if (localVoxelBlock[locId].w_depth == maxW) continue;
			//if (approximateIntegration) if (localVoxelBlock[locId].w_depth != 0) continue;

			pt_model.x = (float)(globalPos.x + x) * voxelSize;
			pt_model.y = (float)(globalPos.y + y) * voxelSize;
			pt_model.z = (float)(globalPos.z + z) * voxelSize;
			pt_model.w = 1.0f;

			// ComputeUpdatedVoxelInfo<TVoxel::hasColorInformation,TVoxel::hasConfidenceInformation, TVoxel>::compute(localVoxelBlock[locId], pt_model, M_d,
			// 	projParams_d, M_rgb, projParams_rgb, mu, maxW, depth, confidence, depthImgSize, rgb, rgbImgSize);

			// Note: The trace for both depth fusion and color fusion is merged together in the following function. This
			// is reasonable because both operations access the same data and can occur in parallel in the pipeline.
			float eta = computeUpdatedVoxelDepthInfoTrace(localVoxelBlock[locId], pt_model, M_d, projParams_d, mu, maxW,
				depth, depthImgSize, entryId, locId, vss);
			if ((eta > mu) || (fabs(eta / mu) > 0.25f))
				continue;
			computeUpdatedVoxelColorInfo(localVoxelBlock[locId], pt_model, M_rgb, projParams_rgb, mu, maxW,
				eta, rgb, rgbImgSize);
		}
	}

	// Write brick trace to file
	ORUtils::Trace::writeTrace(TRACE_FILE_DF_BRICK, count, bss);

	// Write voxel trace to file
	ORUtils::Trace::writeTrace(TRACE_FILE_DF_VOXEL, count, vss);

	count++;
	ORUtils::Trace::enabled_ = (count >= start_frame && count <= end_frame);

	if (count > end_frame) {
		// Exit after last traced frame finishes
		std::exit(EXIT_SUCCESS);
	}
}

template<class TVoxel>
void ITMSceneReconstructionEngine_CPU<TVoxel, ITMVoxelBlockHash>::AllocateSceneFromDepth(ITMScene<TVoxel, ITMVoxelBlockHash> *scene, const ITMView *view,
	const ITMTrackingState *trackingState, const ITMRenderState *renderState, bool onlyUpdateVisibleList, bool resetVisibleList,
	bool useApproximateDepthCheck, bool usePreviousVisibilityList)
{
	Vector2i depthImgSize = view->depth->noDims;
	float voxelSize = scene->sceneParams->voxelSize;

	Matrix4f M_d, invM_d;
	Vector4f projParams_d, invProjParams_d;

	ITMRenderState_VH *renderState_vh = (ITMRenderState_VH*)renderState;
	if (resetVisibleList) renderState_vh->noVisibleEntries = 0;

	if (!usePreviousVisibilityList)
		renderState_vh->Reset();

	M_d = trackingState->pose_d->GetM(); M_d.inv(invM_d);

	projParams_d = view->calib.intrinsics_d.projectionParamsSimple.all;
	invProjParams_d = projParams_d;
	invProjParams_d.x = 1.0f / invProjParams_d.x;
	invProjParams_d.y = 1.0f / invProjParams_d.y;

	float mu = scene->sceneParams->mu;

	float *depth = view->depth->GetData(MEMORYDEVICE_CPU);
	int *voxelAllocationList = scene->localVBA.GetAllocationList();
	int *excessAllocationList = scene->index.GetExcessAllocationList();
	ITMHashEntry *hashTable = scene->index.GetEntries();
	ITMHashSwapState *swapStates = scene->globalCache != NULL ? scene->globalCache->GetSwapStates(false) : 0;
	int *visibleEntryIDs = renderState_vh->GetVisibleEntryIDs();
	uchar *entriesVisibleType = renderState_vh->GetEntriesVisibleType();
	uchar *entriesAllocType = this->entriesAllocType->GetData(MEMORYDEVICE_CPU);
	Vector4s *blockCoords = this->blockCoords->GetData(MEMORYDEVICE_CPU);
	int noTotalEntries = scene->index.noTotalEntries;

	bool useSwapping = scene->globalCache != NULL;

	float oneOverVoxelSize = 1.0f / (voxelSize * SDF_BLOCK_SIZE);

	int lastFreeVoxelBlockId = scene->localVBA.lastFreeBlockId;
	int lastFreeExcessListId = scene->index.GetLastFreeExcessListId();

	int noVisibleEntries = 0;

	// Trace stream
	std::stringstream ss;

	memset(entriesAllocType, 0, noTotalEntries);

	for (int i = 0; i < renderState_vh->noVisibleEntries; i++)
		entriesVisibleType[visibleEntryIDs[i]] = 3; // visible at previous frame and unstreamed

	//build hashVisibility
#ifdef WITH_OPENMP
	#pragma omp parallel for
#endif
	for (int locId = 0; locId < depthImgSize.x*depthImgSize.y; locId++)
	{
		int y = locId / depthImgSize.x;
		int x = locId - y * depthImgSize.x;
		if (useApproximateDepthCheck)
			buildHashAllocAndVisibleTypePointOnly(entriesAllocType, entriesVisibleType, x, y, blockCoords, depth, invM_d,
				invProjParams_d, mu, depthImgSize, oneOverVoxelSize, hashTable, scene->sceneParams->viewFrustum_min,
				scene->sceneParams->viewFrustum_max);
		else
			buildHashAllocAndVisibleTypePPTrace(entriesAllocType, entriesVisibleType, x, y, blockCoords, depth, invM_d,
				invProjParams_d, mu, depthImgSize, oneOverVoxelSize, hashTable, scene->sceneParams->viewFrustum_min,
				scene->sceneParams->viewFrustum_max, locId, ss);

	}

	if (onlyUpdateVisibleList) useSwapping = false;
	if (!onlyUpdateVisibleList)
	{
		//allocate
		for (int targetIdx = 0; targetIdx < noTotalEntries; targetIdx++)
		{
			int vbaIdx, exlIdx;
			unsigned char hashChangeType = entriesAllocType[targetIdx];

			switch (hashChangeType)
			{
			case 1: //needs allocation, fits in the ordered list
				vbaIdx = lastFreeVoxelBlockId; lastFreeVoxelBlockId--;

				if (vbaIdx >= 0) //there is room in the voxel block array
				{
					Vector4s pt_block_all = blockCoords[targetIdx];

					ITMHashEntry hashEntry;
					hashEntry.pos.x = pt_block_all.x; hashEntry.pos.y = pt_block_all.y; hashEntry.pos.z = pt_block_all.z;
					hashEntry.ptr = voxelAllocationList[vbaIdx];
					hashEntry.offset = 0;

					hashTable[targetIdx] = hashEntry;
				}
				else
				{
					// Mark entry as not visible since we couldn't allocate it but buildHashAllocAndVisibleTypePP changed its state.
					entriesVisibleType[targetIdx] = 0;

					// Restore previous value to avoid leaks.
					lastFreeVoxelBlockId++;
				}

				break;
			case 2: //needs allocation in the excess list
				vbaIdx = lastFreeVoxelBlockId; lastFreeVoxelBlockId--;
				exlIdx = lastFreeExcessListId; lastFreeExcessListId--;

				if (vbaIdx >= 0 && exlIdx >= 0) //there is room in the voxel block array and excess list
				{
					Vector4s pt_block_all = blockCoords[targetIdx];

					ITMHashEntry hashEntry;
					hashEntry.pos.x = pt_block_all.x; hashEntry.pos.y = pt_block_all.y; hashEntry.pos.z = pt_block_all.z;
					hashEntry.ptr = voxelAllocationList[vbaIdx];
					hashEntry.offset = 0;

					int exlOffset = excessAllocationList[exlIdx];

					hashTable[targetIdx].offset = exlOffset + 1; //connect to child

					hashTable[SDF_BUCKET_NUM + exlOffset] = hashEntry; //add child to the excess list

					entriesVisibleType[SDF_BUCKET_NUM + exlOffset] = 1; //make child visible and in memory
				}
				else
				{
					// No need to mark the entry as not visible since buildHashAllocAndVisibleTypePP did not mark it.
					// Restore previous value to avoid leaks.
					lastFreeVoxelBlockId++;
					lastFreeExcessListId++;
				}

				break;
			}
		}
	}

	//build visible list
	for (int targetIdx = 0; targetIdx < noTotalEntries; targetIdx++)
	{
		unsigned char hashVisibleType = entriesVisibleType[targetIdx];
		const ITMHashEntry &hashEntry = hashTable[targetIdx];

		if (hashVisibleType == 3)
		{
			bool isVisibleEnlarged, isVisible;

			if (useSwapping)
			{
				checkBlockVisibility<true>(isVisible, isVisibleEnlarged, hashEntry.pos, M_d, projParams_d, voxelSize, depthImgSize);
				if (!isVisibleEnlarged) hashVisibleType = 0;
			} else {
				checkBlockVisibility<false>(isVisible, isVisibleEnlarged, hashEntry.pos, M_d, projParams_d, voxelSize, depthImgSize);
				if (!isVisible) { hashVisibleType = 0; }
			}
			entriesVisibleType[targetIdx] = hashVisibleType;
		}

		if (useSwapping)
		{
			if (hashVisibleType > 0 && swapStates[targetIdx].state != 2) swapStates[targetIdx].state = 1;
		}

		if (hashVisibleType > 0)
		{
			// Visibility list store instruction with magic instruction ID
			visibleEntryIDs[noVisibleEntries] = targetIdx;
			ORUtils::Trace::writeMemoryInstruction(ss, noVisibleEntries, 786, "store", "global", "vis", std::addressof(visibleEntryIDs[noVisibleEntries]));
			noVisibleEntries++;
		}

#if 0
		// "active list", currently disabled
		if (hashVisibleType == 1)
		{
			activeEntryIDs[noActiveEntries] = targetIdx;
			noActiveEntries++;
		}
#endif
	}

	//reallocate deleted ones from previous swap operation
	if (useSwapping)
	{
		for (int targetIdx = 0; targetIdx < noTotalEntries; targetIdx++)
		{
			int vbaIdx;
			ITMHashEntry hashEntry = hashTable[targetIdx];

			if (entriesVisibleType[targetIdx] > 0 && hashEntry.ptr == -1)
			{
				vbaIdx = lastFreeVoxelBlockId; lastFreeVoxelBlockId--;
				if (vbaIdx >= 0) hashTable[targetIdx].ptr = voxelAllocationList[vbaIdx];
				else lastFreeVoxelBlockId++; // Avoid leaks
			}
		}
	}

	renderState_vh->noVisibleEntries = noVisibleEntries;

	scene->localVBA.lastFreeBlockId = lastFreeVoxelBlockId;
	scene->index.SetLastFreeExcessListId(lastFreeExcessListId);

	// Write visibility check pixel trace to file
	ORUtils::Trace::writeTrace(TRACE_FILE_VC_PIXEL, count, ss);
}

template<class TVoxel>
ITMSceneReconstructionEngine_CPU<TVoxel,ITMPlainVoxelArray>::ITMSceneReconstructionEngine_CPU(void)
{}

template<class TVoxel>
ITMSceneReconstructionEngine_CPU<TVoxel,ITMPlainVoxelArray>::~ITMSceneReconstructionEngine_CPU(void)
{}

template<class TVoxel>
void ITMSceneReconstructionEngine_CPU<TVoxel,ITMPlainVoxelArray>::ResetScene(ITMScene<TVoxel, ITMPlainVoxelArray> *scene)
{
	int numBlocks = scene->index.getNumAllocatedVoxelBlocks();
	int blockSize = scene->index.getVoxelBlockSize();

	TVoxel *voxelBlocks_ptr = scene->localVBA.GetVoxelBlocks();
	for (int i = 0; i < numBlocks * blockSize; ++i) voxelBlocks_ptr[i] = TVoxel();
	int *vbaAllocationList_ptr = scene->localVBA.GetAllocationList();
	for (int i = 0; i < numBlocks; ++i) vbaAllocationList_ptr[i] = i;
	scene->localVBA.lastFreeBlockId = numBlocks - 1;
}

template<class TVoxel>
void ITMSceneReconstructionEngine_CPU<TVoxel, ITMPlainVoxelArray>::AllocateSceneFromDepth(ITMScene<TVoxel, ITMPlainVoxelArray> *scene, const ITMView *view,
	const ITMTrackingState *trackingState, const ITMRenderState *renderState, bool onlyUpdateVisibleList, bool resetVisibleList,
	bool useApproximateDepthCheck, bool usePreviousVisibilityList)
{}

template<class TVoxel>
void ITMSceneReconstructionEngine_CPU<TVoxel, ITMPlainVoxelArray>::IntegrateIntoScene(ITMScene<TVoxel, ITMPlainVoxelArray> *scene, const ITMView *view,
	const ITMTrackingState *trackingState, const ITMRenderState *renderState)
{
	Vector2i rgbImgSize = view->rgb->noDims;
	Vector2i depthImgSize = view->depth->noDims;
	float voxelSize = scene->sceneParams->voxelSize;

	Matrix4f M_d, M_rgb;
	Vector4f projParams_d, projParams_rgb;

	M_d = trackingState->pose_d->GetM();
	if (TVoxel::hasColorInformation) M_rgb = view->calib.trafo_rgb_to_depth.calib_inv * M_d;

	projParams_d = view->calib.intrinsics_d.projectionParamsSimple.all;
	projParams_rgb = view->calib.intrinsics_rgb.projectionParamsSimple.all;

	float mu = scene->sceneParams->mu; int maxW = scene->sceneParams->maxW;

	float *depth = view->depth->GetData(MEMORYDEVICE_CPU);
	Vector4u *rgb = view->rgb->GetData(MEMORYDEVICE_CPU);
	TVoxel *voxelArray = scene->localVBA.GetVoxelBlocks();

	const ITMPlainVoxelArray::IndexData *arrayInfo = scene->index.getIndexData();

	bool stopIntegratingAtMaxW = scene->sceneParams->stopIntegratingAtMaxW;
	//bool approximateIntegration = !trackingState->requiresFullRendering;

#ifdef WITH_OPENMP
	#pragma omp parallel for
#endif
	for (int locId = 0; locId < scene->index.getVolumeSize().x*scene->index.getVolumeSize().y*scene->index.getVolumeSize().z; ++locId)
	{
		int z = locId / (scene->index.getVolumeSize().x*scene->index.getVolumeSize().y);
		int tmp = locId - z * scene->index.getVolumeSize().x*scene->index.getVolumeSize().y;
		int y = tmp / scene->index.getVolumeSize().x;
		int x = tmp - y * scene->index.getVolumeSize().x;
		Vector4f pt_model;

		if (stopIntegratingAtMaxW) if (voxelArray[locId].w_depth == maxW) continue;
		//if (approximateIntegration) if (voxelArray[locId].w_depth != 0) continue;

		pt_model.x = (float)(x + arrayInfo->offset.x) * voxelSize;
		pt_model.y = (float)(y + arrayInfo->offset.y) * voxelSize;
		pt_model.z = (float)(z + arrayInfo->offset.z) * voxelSize;
		pt_model.w = 1.0f;

		ComputeUpdatedVoxelInfo<TVoxel::hasColorInformation, TVoxel::hasConfidenceInformation, TVoxel>::compute(voxelArray[locId], pt_model, M_d, projParams_d, M_rgb, projParams_rgb, mu, maxW,
			depth, depthImgSize, rgb, rgbImgSize);
	}
}
