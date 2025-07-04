#!/bin/bash

# ARGoS Multiple Run Script
# This script runs the simulation 10 times with different random seeds
# and extracts the "Resource Collected:" lines from the output

CONFIG_FILE="experiments/Clustered_CPFA_r64_tag512_16by16.xml"
OUTPUT_FILE="results_18_mins_80_robots.txt"
TEMP_CONFIG="temp_80.xml"

# Array of random seeds to use
SEEDS=(123456 789012 345678 901234 567890 135791 246802 864209 751936 428573 
       613579 982034 704321 319864 580246 162738 947120 830591 204869 678345  
       793624 115320 926481 307159 481206 693018 859473 374829 142607 620591)

# Clear previous results
> "$OUTPUT_FILE"

echo "Starting 10 simulation runs with different random seeds..."
echo "Results will be saved to: $OUTPUT_FILE"
echo ""

for i in {0..9}; do
    SEED=${SEEDS[$i]}
    RUN_NUMBER=$((i + 1))
    
    echo "Run $RUN_NUMBER: Using random seed $SEED"
    
    # Create temporary config file with new random seed
    sed "s/random_seed=\"[0-9]*\"/random_seed=\"$SEED\"/" "$CONFIG_FILE" > "$TEMP_CONFIG"
    
    # Run ARGoS simulation and capture output
    echo "=== Run $RUN_NUMBER (Seed: $SEED) ===" >> "$OUTPUT_FILE"
    
    # Run the simulation and filter for "Resource Collected:" lines
    argos3 -c "$TEMP_CONFIG" 2>&1 | grep "Resource Collected:" | tee -a "$OUTPUT_FILE"
    
    echo "" >> "$OUTPUT_FILE"
    
    echo "Run $RUN_NUMBER completed"
    echo ""
done

# Clean up temporary file
rm -f "$TEMP_CONFIG"

echo "All runs completed!"
echo "Results summary:"
echo "=================="

# Extract and display just the Resource Collected values
echo "Run | Seed     | Resources Collected"
echo "----|----------|-------------------"

run_num=1
for seed in "${SEEDS[@]}"; do
    # Extract the resource count for this run
    resource_count=$(grep -A1 "Run $run_num (Seed: $seed)" "$OUTPUT_FILE" | grep "Resource Collected:" | awk '{print $NF}')
    printf "%2d  | %8s | %s\n" "$run_num" "$seed" "$resource_count"
    ((run_num++))
done

echo ""
echo "Detailed results saved in: $OUTPUT_FILE"