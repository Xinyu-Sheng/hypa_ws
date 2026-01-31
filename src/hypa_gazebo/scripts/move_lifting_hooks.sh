#!/bin/bash

# Script to move HYPA lifting hooks to multiple heights
# Controls all four prismatic joints (front-left, front-right, back-left, back-right)

# Joint topics for all four lifting hooks
JOINT_TOPICS=(
    "/model/hypa/joint/prismatic_lifting_hook_fl/0/cmd_pos"  # front-left
    "/model/hypa/joint/prismatic_lifting_hook_fr/0/cmd_pos"  # front-right
    "/model/hypa/joint/prismatic_lifting_hook_bl/0/cmd_pos"  # back-left
    "/model/hypa/joint/prismatic_lifting_hook_br/0/cmd_pos"  # back-right
)

# Function to move all hooks to specific position
move_hooks_to_position() {
    local position=$1
    local wait_time=${2:-2}  # default wait time is 2 seconds
    
    echo "Moving all hooks to position: $position"
    
    # Build and execute commands for all joints in parallel
    local commands=""
    for topic in "${JOINT_TOPICS[@]}"; do
        commands+="gz topic -t $topic -m gz.msgs.Double -p \"data: $position\" & "
    done
    commands+="wait"
    
    eval $commands
    echo "Successfully moved hooks to $position"
    sleep $wait_time
}

# Function to move through sequence of heights
move_to_heights() {
    local heights=("$@")
    echo "Starting movement sequence through heights: ${heights[*]}"
    
    for i in "${!heights[@]}"; do
        echo -e "\nStep $((i+1))/${#heights[@]}: Moving to ${heights[i]}"
        move_hooks_to_position "${heights[i]}"
    done
    
    echo -e "\nMovement sequence completed!"
}

# Test movement function
test_movement() {
    local test_heights=(0.0 0.5 1.0 1.5 2.0 2.5 3.0 2.5 2.0 1.5 1.0 0.5 0.0)
    move_to_heights "${test_heights[@]}"
}

# Main script logic
main() {
    if [ $# -eq 0 ]; then
        echo "Usage:"
        echo "  $0 test                    # Run test sequence"
        echo "  $0 <height>                # Move to specific height"
        echo "  $0 <h1> <h2> <h3> ...      # Move through sequence"
        echo ""
        echo "Examples:"
        echo "  $0 1.5"
        echo "  $0 0.0 1.0 2.0 1.0 0.0"
        echo "  $0 test"
        return 1
    fi
    
    if [ "$1" = "test" ]; then
        test_movement
    else
        # Convert all arguments to heights array
        local heights=("$@")
        if [ ${#heights[@]} -eq 1 ]; then
            move_hooks_to_position "${heights[0]}"
        else
            move_to_heights "${heights[@]}"
        fi
    fi
}

# Run main function with all arguments
main "$@"