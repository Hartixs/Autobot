#!/usr/bin/env python3
import math
import subprocess
import time

LOCATIONS = {
    'pantry': (1990.00, 638.36),
    'lounge': (2438.39, 471.24),
    'supplies': (770.88, 383.43),
    'hardware_2': (2474.55, 885.64),
    'coe': (631.56, 587.89),
    'patrol_A1': (1191.44, 824.46),
    'patrol_A2': (2238.47, 477.12),
    'patrol_B': (943.63, 1273.43),
    'patrol_D1': (1991.12, 812.87),
    'patrol_D2': (1210.54, 365.25)
}

ROBOTS = {
    "tinyRobot1": {"pos": (1232.42, 658.57), "free_after": 0},
    "tinyRobot2": {"pos": (2412.58, 627.50), "free_after": 0}
}

SIMULATED_SPEED = 30.0 

def calculate_distance(pos1, pos2):
    return math.sqrt((pos2[0] - pos1[0])**2 + (pos2[1] - pos1[1])**2)

def interactive_dashboard():
    print("\n" + "="*50)
    print(" PREDICTIVE SWARM AUCTION INITIALIZED")
    print(" Press Ctrl+C at any time to exit.")
    print("="*50)

    try:
        while True:
            current_time = time.time()
            
            print("\nAVAILABLE LOCATIONS:")
            print(", ".join(LOCATIONS.keys()))
            print("-" * 50)

            target_name = input("Enter destination (or type 'exit'): ").strip().lower()
            
            if target_name == 'exit':
                break
            if target_name not in LOCATIONS:
                print("Invalid location. Please choose from the list.")
                continue

            target_pos = LOCATIONS[target_name]

            print(f"\nAUCTION STARTED: Go to {target_name.upper()}")
            best_bid = float('inf')
            winner = None

            # EVERY robot gets to bid, even if busy!
            for robot, state in ROBOTS.items():
                
                # 1. Calculate time penalty if currently busy
                time_penalty = max(0.0, state["free_after"] - current_time)
                
                # 2. Calculate travel time from its FUTURE position to the new task
                travel_time = calculate_distance(state["pos"], target_pos) / SIMULATED_SPEED
                
                # 3. The Bid is the total Estimated Time of Arrival (ETA)
                eta_bid = time_penalty + travel_time
                
                if time_penalty > 0:
                    print(f"  [BID] {robot} is BUSY. Bids ETA: {eta_bid:.1f}s (Penalty: {time_penalty:.1f}s)")
                else:
                    print(f"  [BID] {robot} is FREE. Bids ETA: {eta_bid:.1f}s")

                if eta_bid < best_bid:
                    best_bid = eta_bid
                    winner = robot
            
            time.sleep(1) 
            
            print("  ---")
            print(f"WINNER: {winner} claims the task!")
            
            # Note: Added '-R {winner}' to explicitly force Open-RMF to use the winning robot
            command = f"ros2 run rmf_demos_tasks dispatch_go_to_place -p {target_name} -R {winner} --use_sim_time"
            subprocess.Popen(command, shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            print(f"Dispatching {winner} to {target_name}...")

            # Update the Winner's State Memory for the next auction
            # We add 5 seconds to account for docking/turning time
            ROBOTS[winner]["pos"] = target_pos 
            ROBOTS[winner]["free_after"] = max(current_time, ROBOTS[winner]["free_after"]) + (best_bid - time_penalty) + 5.0

            print("="*50)
            time.sleep(1) 

    except KeyboardInterrupt:
        print("\n\nDashboard Shutting Down.")

if __name__ == "__main__":
    interactive_dashboard()
