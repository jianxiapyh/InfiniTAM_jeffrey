sequences=('large_loop_1' 'mannequin_1' 'mannequin_face_2' 'plant_5' 'plant_scene_1' 'repetitive' 'sofa_1' 'table_3')
frequencies=('2' '5' '7.5' '10' '15' '30')

file=$1
dataset_folder="/home/huzaifa2/workspace/xr/datasets/ETH3D"

if [ -z $file ];
then
	echo "Usage: ./ETH3D.sh <pose_file>"
	echo "<pose_file>: Associated pose file"
	exit 1
fi

# ICP
export useICP=false

# Approximations
export approxDepthCheck=false
export usePrevList=true

# Fusion
export freqMode=constant

# Raycasting
export decoupleRaycasting=true
export raycastingFrequency=1.0

# Run all sequences with all frequencies
for sequence in ${sequences[@]}
do
	for frequency in ${frequencies[@]}
	do
		# Set fusion frequency
		export fusionFrequency=$frequency

		# Sequence fully associated with specified pose
		../../build/Apps/InfiniTAM_cli/InfiniTAM_cli ../calibration/ETH3D.txt ${dataset_folder}/ETH3D-$sequence/poses/$file

		# Clean mesh
		time python3.8 ../misc/clean_mesh.py --mesh ITM-BE.obj --smoothen

		# Rename output files
		mv ITM-BE_cleaned.obj eth3d_${sequence}_${freqMode}_${fusionFrequency}.obj
		mv execution_data.csv execution_eth3d_${sequence}_${freqMode}_${fusionFrequency}.csv
		mv controller_data.csv controller_eth3d_${sequence}_${freqMode}_${fusionFrequency}.csv
		mv brick_data.csv brick_eth3d_${sequence}_${freqMode}_${fusionFrequency}.csv
		mv voxel_data.csv voxel_eth3d_${sequence}_${freqMode}_${fusionFrequency}.csv
	done
done
