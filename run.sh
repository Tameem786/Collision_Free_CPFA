#!/bin/bash

# ARGoS Multi-Run Script
# Runs the configuration file 10 times with different experiment lengths

CONFIG_FILE="experiments/Clustered_CPFA_r64_tag512_16by16.xml"
ORIGINAL_LENGTH=900

# Check if config file exists
if [ ! -f "$CONFIG_FILE" ]; then
    echo "Error: Configuration file '$CONFIG_FILE' not found!"
    exit 1
fi

# Create a backup of the original file
cp "$CONFIG_FILE" "${CONFIG_FILE}.backup"

echo "Starting 10 runs with different experiment lengths..."
echo "Original length: $ORIGINAL_LENGTH"
echo "=================================="

for i in {1..10}; do
    # Generate a new experiment length
    NEW_LENGTH=$((ORIGINAL_LENGTH + (100 * i)))
    
    echo "Run $i: Using experiment length $NEW_LENGTH"
    
    # Replace the length in the config file
    sed -i "s/length=\"[0-9]*\"/length=\"$NEW_LENGTH\"/" "$CONFIG_FILE"
    
    # Run ARGoS with the modified configuration
    echo "  Executing: argos3 -c $CONFIG_FILE"
    argos3 -c "$CONFIG_FILE"
    
    # Check if the run was successful
    if [ $? -eq 0 ]; then
        echo "  Run $i completed successfully"
    else
        echo "  Run $i failed!"
    fi
    
    echo "  --------------------------------"
done

# Restore the original configuration file
mv "${CONFIG_FILE}.backup" "$CONFIG_FILE"

echo "All runs completed!"
echo "Original configuration file restored."