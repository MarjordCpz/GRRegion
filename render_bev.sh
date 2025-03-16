#!/bin/bash

#SBATCH --partition=gpu_4090  
#SBATCH --nodes=1             
#SBATCH --gres=gpu:2          
#SBATCH --cpus-per-task=12  
#SBATCH --time=14-00:00:00    
#SBATCH --array=0-3
#SBATCH --output=/ailab/user/caopeizhou/projects/GRRegion/logs/one_scene_render/110_119/%j.out       
#SBATCH --error=/ailab/user/caopeizhou/projects/GRRegion/logs/one_scene_render/110_119/%j.err 


if [ $# -ne 2 ]; then
    echo "Usage: $0 part_index usd_fodler"
    exit 1
fi
part_index=$1
usd_fodler=$2


task_id=$SLURM_ARRAY_TASK_ID

start_id=$((task_id*25))
end_id=$(((task_id+1)*25))

# start_id=52
# end_id=75

echo ${start_id}
echo ${end_id}

for ((i = start_id; i < end_id; i += 1)); do
    /ailab/user/caopeizhou/apps/isaac-sim-4.1.0/python.sh getmaps.py \
        --scene_id $i \
        --part $part_index \
        --usd_folder $usd_fodler
    wait
done

