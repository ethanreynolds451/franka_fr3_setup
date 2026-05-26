# Use the Franka API to release control

import requests
import urllib3
import os
import yaml
import time

# *** User Configuration ****************************************

# Default parameters used if no configuration file is provided
ROBOT_IP = "192.168.1.1"
USER = "snail-lab"
PASSWORD ="password123"

# Search directories
config_dir = os.path.dirname(os.path.abspath(__file__))
token_save_dir = os.path.expanduser("~/Desktop")

# ***************************************************************

# Generate full file paths
config_file = os.path.join(config_dir, "fci_config.yaml")
token_file = os.path.join(token_save_dir, "control_token.txt")

# Suppress network warnings for requests to API with self-signed certificates
urllib3.disable_warnings()

# Load custom configuration parameters form a file if it exists
# If not found or invalid, use default parameters defined above
def load_config():
    if os.path.exists(config_file):
        config = {}
        # Load parameters from the file
        try:
            with open(config_file, "r") as f:
                config = yaml.safe_load(f)
            print(f"Configuration loaded from {config_file}")
            # Allow modificatino of global variables
            global ROBOT_IP, USER, PASSWORD
            new_ip = config.get("robot_ip", ROBOT_IP)
            new_user = config.get("user", USER)
            new_password = config.get("password", PASSWORD)
            if new_ip:
                ROBOT_IP = new_ip
            if new_user:
                USER = new_user
            if new_password:
                PASSWORD = new_password
            print(f"Using parameters ROBOT_IP={ROBOT_IP}, USER={USER}, PASSWORD={PASSWORD}")
        except IOError as e:
            print(f"Error loading configuration: {e}")
            print("Using default parameters.")
    else:
        print(f"No configuration file found at {config_file}. Using default parameters.")

# Load the Franka control token from file
# If no file found, prompt the user to enter it manually
# Token is required to exit FCI and release control, but not to lock the arm
def get_token():
    if os.path.exists(token_file):
        try:
            with open(token_file, "r") as f:
                token = f.read().strip()
            print(f"Control token loaded from {token_file}")
            return token
        except IOError as e:
            print(f"Error reading control token: {e}")
            print(f"No control token found at {token_file}. Please enter manually:")
            token = input("Control Token: ").strip()
    else:
        print(f"No control token found at {token_file}. Please enter manually:")
        token = input("Control Token: ").strip()
    if token:
        return token
    else:
        print("No control token provided. Arm will lock but control cannot be released.")
        return None

# Use the API to release the control token 
def release_token(token):
    url = f"https://{ROBOT_IP}/api/system/control-token:release"
    try: 
        response = requests.post(
            url,
            auth=(USER, PASSWORD),
            headers={
                "X-Control-Token": token
            },
            verify=False
        )
        response.raise_for_status()
        return True
    except requests.exceptions.RequestException as e:
        print(f"Error releasing control token: {e}")
        return None

def delete_token_file():
    if os.path.exists(token_file):
        try:
            os.remove(token_file)
            print(f"Control token file {token_file} deleted.")
        except IOError as e:
            print(f"Error deleting control token file: {e}")
    else:
        print(f"No control token file found at {token_file}. Nothing to delete.")

if __name__ == "__main__":
    load_config()   # No return, feedback handled internally
    print("Searching for control token...")
    active_token = get_token()
    if active_token:
        print(f"Control token found: {active_token}")
        print("Releasing control token...")
        if release_token(active_token):
            print("Control token released successfully.")
            delete_token_file()
            print("Deleting control token file...")
            print("Exiting in 10 seconds...")
            time.sleep(10)  # Give the user time to read the messages
            # Exit with success code
            exit(0)
        else:
            print("Failed to release control token. Release can be forced through the DESK interface with physical access to the robot.")
    else:
        print("No control token found. Release can be forced through the DESK interface with physical access to the robot.")
        
    print("Exiting in 10 seconds...")
    time.sleep(10)  # Give the user time to read the messages
    # Exit with failure code
    exit(1)