sequences=('0000' '0001' '0002' '0004' '0005' '0010' '0101')

seq=$1
file=$2
dataset_folder="/home/huzaifa2/workspace/xr/datasets/ScanNet"

if [ -z $seq ] || [ -z $file ];
then
	echo "Usage: ./ScanNet.sh <sequence> <pose_file>"
	echo "<sequence>: One of the following sequences:"

	for element in ${sequences[@]}
	do
		echo -e "\t$element"
	done
	echo ""

	echo "<pose_file>: Associated pose file"
	exit 1
fi

# ICP
export useICP=false

# Approximations
export approxDepthCheck=false
export usePrevList=false

# Fusion
export freqMode=none
export fusionFrequency=30.0

# Raycasting
export decoupleRaycasting=true
export raycastingFrequency=1.0

# Run InfiniTAM
../../build/Apps/InfiniTAM/InfiniTAM ${dataset_folder}/scene${seq}/calibration.txt ${dataset_folder}/scene${seq}/poses/$file

# Clean mesh
time python3.8 ../misc/clean_mesh.py --mesh ITM-ICP.obj --smoothen

# Rename output files
mv ITM-ICP_cleaned.obj scannet_${seq}_${freqMode}_${fusionFrequency}.obj
mv execution_data.csv execution_scannet_${seq}_${freqMode}_${fusionFrequency}.csv
mv controller_data.csv controller_scannet_${seq}_${freqMode}_${fusionFrequency}.csv
mv brick_data.csv brick_scannet_${seq}_${freqMode}_${fusionFrequency}.csv
mv voxel_data.csv voxel_scannet_${seq}_${freqMode}_${fusionFrequency}.csv
