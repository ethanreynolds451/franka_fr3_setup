# Use the Franka API to take control, unlock the arm, and start the FCI

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

# Time in seconds to wait for control token to be released
# If another user is curently in control, they will need to release the token
token_request_timeout = 10

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
        # Just using the catch-all exception for simplicity
        except Exception as e:
            print(f"Error loading configuration: {e}")
            print("Using default parameters.")
    else:
        print(f"No configuration file found at {config_file}. Using default parameters.")

# Check if the control token file exists and contains a valid token
def have_control_token():
    if os.path.exists(token_file):
        try:
            with open(token_file, "r") as f:
                token = f.read().strip()
            if token:
                return True
        except IOError as e:
            print(f"Error reading control token file: {e}")
    return False

# Use the API to request control token, which is required to unlock the arm and start FCI
# If another user currently has control, this request will wait until the token is released or the request times out
def get_token():
    url = f"https://{ROBOT_IP}/api/system/control-token:take"
    try: 
        response = requests.post(
            url,
            auth=(USER, PASSWORD),
            json= {
                "owner": USER
            },
            verify=False,
            timeout=token_request_timeout
        )
        response.raise_for_status()
        return response.json()["token"]
    # Specific message for timeout 
    except requests.exceptions.Timeout:
        print("Token request timed out. Another user may currently have control. Please release token and try again.")
        return None
    # Any other request exceptions
    except requests.exceptions.RequestException as e:
        print(f"Error obtaining control token: {e}")
        return None

# Save the token to a file for later use
# May be used by other scripts and is needed ot release control
def save_token(token):
    try:
        with open(token_file, "w") as f:
            f.write(token)
        return True
    except IOError as e:
        print(f"Error saving control token: {e}")
        return False

# Use the API to unlock the arm
# The response waits until the arm is physically released, causing a slight delay
def unlock_arm(token):
    url = f"https://{ROBOT_IP}/api/arm/joints:unlock"
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
        print(f"Error unlocking arm: {e}")
        return False

# Use the API to enable FCI mode
# This allows for direct control through ROS and other interfaces
def start_fci(token):
    url = f"https://{ROBOT_IP}/api/fci:activate"
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
        print(f"Error starting FCI: {e}")
        return False

# Use the API to make sure that FCI was successfully started
def check_fci_status():
    url = f"https://{ROBOT_IP}/api/fci"
    try:
        response = requests.get(
            url,
            auth=(USER, PASSWORD),
            verify=False
        )
        response.raise_for_status()
        status = response.json().get("status", "unknown")
        return status == "Active"
    except requests.exceptions.RequestException as e:
        print(f"Error checking FCI status: {e}")
        return False

if __name__ == "__main__":
    load_config()   # No return, feedback handled internally 
    if not have_control_token():
        print("Requesting control token...")
        active_token = get_token()
    else:
        print(f"Control token already exists in {token_file}. Using existing token.")
        with open(token_file, "r") as f:
            active_token = f.read().strip()
    close_window = True
    if active_token:
        print(f"Control token: {active_token}")
        if save_token(active_token):
            print(f"Control token saved to {token_file}")
        else:
            print("The window will remain open so you can copy the token manually.")
            # Don't exit if the control token wasn't saved since the user will have to copy it manually
            close_window = False
        print("Unlocking arm. This may take a few seconds...")
        if unlock_arm(active_token):
            print("Arm unlocked successfully.")
            print("Starting FCI...")
            if start_fci(active_token):
                print("Checking FCI status...")
                if check_fci_status():
                    print("FCI started successfully.")
                    if close_window:
                        print("Exiting in 10 seconds...")
                        time.sleep(10)  # Give the user time to read the messages
                        # Exit with success code
                        exit(0)
                else:
                    print("Failed to start FCI.")
            else:
                print("Failed to start FCI.")
        else:
            print("Failed to unlock arm. Unable to start FCI.")  
    else:
        print("Failed to obtain control token. Unable to start FCI.")
    if close_window:
        print("Exiting in 10 seconds...")
        time.sleep(10)  # Give the user time to read the messages
        # Exit with failure code
        exit(1)