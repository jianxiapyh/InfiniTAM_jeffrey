sequences=('0000' '0002' '0004' '0005' '0010' '0101')
frequencies=('2' '5' '7.5' '10' '15' '30')

file=$1
dataset_folder="/home/huzaifa2/workspace/xr/datasets/ScanNet"

if [ -z $file ];
then
	echo "Usage: ./ScanNet.sh <pose_file>"
	echo "<pose_file>: Associated pose file"
	exit 1
fi

# ICP
export useICP=false

# Approximations
export approxDepthCheck=false
export usePrevList=false

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

		# Run InfiniTAM
		../../build/Apps/InfiniTAM_cli/InfiniTAM_cli ${dataset_folder}/scene${sequence}/calibration.txt ${dataset_folder}/scene${sequence}/poses/$file

		# Clean mesh
		time python3.8 ../misc/clean_mesh.py --mesh ITM-BE.obj --smoothen

		# Rename output files
		mv ITM-BE_cleaned.obj scannet_${sequence}_${freqMode}_${fusionFrequency}.obj
		mv execution_data.csv execution_scannet_${sequence}_${freqMode}_${fusionFrequency}.csv
		mv controller_data.csv controller_scannet_${sequence}_${freqMode}_${fusionFrequency}.csv
		mv brick_data.csv brick_scannet_${sequence}_${freqMode}_${fusionFrequency}.csv
		mv voxel_data.csv voxel_scannet_${sequence}_${freqMode}_${fusionFrequency}.csv
	done
done
