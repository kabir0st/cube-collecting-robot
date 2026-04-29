#!/bin/bash

# Configuration
SOURCE_DIR="./labs-solution"
ZIP_FILE="labs-solution.zip"
REMOTE_USER="pi"
REMOTE_HOST="10.228.190.142"
REMOTE_PASSWORD="pi"
REMOTE_PATH="/home/pi"

# Check if sshpass is installed
if ! command -v sshpass &> /dev/null; then
    echo "sshpass is not installed. Installing..."
    sudo pacman -S --noconfirm sshpass
fi

# Remove existing zip file if it exists
if [ -f "$ZIP_FILE" ]; then
    rm "$ZIP_FILE"
fi

# Create zip file
echo "Creating zip archive of $SOURCE_DIR..."
zip -r "$ZIP_FILE" "$SOURCE_DIR"

# Transfer file using scp with sshpass
echo "Transferring to ${REMOTE_USER}@${REMOTE_HOST}..."
sshpass -p "$REMOTE_PASSWORD" scp "$ZIP_FILE" "${REMOTE_USER}@${REMOTE_HOST}:${REMOTE_PATH}"

# Execute remote commands using ssh with sshpass
echo "Extracting on remote host..."
sshpass -p "$REMOTE_PASSWORD" ssh "${REMOTE_USER}@${REMOTE_HOST}" << 'EOF'
    # Remove existing labs-solution folder if it exists
    if [ -d "/home/pi/labs-solution" ]; then
        echo "Removing existing labs-solution folder..."
        rm -rf /home/pi/labs-solution
    fi
    
    # Extract the new zip file
    cd /home/pi
    unzip -o labs-solution.zip
    
    # Clean up zip file
    rm labs-solution.zip
    
    echo "Transfer complete!"
EOF

# Clean up local zip file
rm "$ZIP_FILE"
echo "Local cleanup done."
