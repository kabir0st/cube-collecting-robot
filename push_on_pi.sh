#!/bin/bash

# Configuration
ZIP_FILE="group-6.zip"
REMOTE_USER="pi"
REMOTE_HOST="192.168.0.37"
REMOTE_PASSWORD="pi"
REMOTE_PATH="/home/pi"
REMOTE_DIR="group-6"

# Check if sshpass is installed
if ! command -v sshpass &> /dev/null; then
    echo "sshpass is not installed. Installing..."
    sudo pacman -S --noconfirm sshpass
fi

# Remove existing zip file if it exists
if [ -f "$ZIP_FILE" ]; then
    rm "$ZIP_FILE"
fi

echo "Creating zip archive $ZIP_FILE..."
zip -r "$ZIP_FILE" main.cpp src include Makefile libraries config.yml

# Transfer file using scp with sshpass
echo "Transferring to ${REMOTE_USER}@${REMOTE_HOST}..."
sshpass -p "$REMOTE_PASSWORD" scp "$ZIP_FILE" "${REMOTE_USER}@${REMOTE_HOST}:${REMOTE_PATH}"

# Execute remote commands using ssh with sshpass
echo "Extracting on remote host into ${REMOTE_PATH}/${REMOTE_DIR}..."
sshpass -p "$REMOTE_PASSWORD" ssh "${REMOTE_USER}@${REMOTE_HOST}" << EOF
    # Keep existing ${REMOTE_DIR} (and its build/ cache) intact; just
    # overwrite changed source files. -o makes unzip overwrite without
    # prompting. Files deleted locally will linger on the Pi until a
    # manual cleanup or 'make clean'.
    mkdir -p "${REMOTE_PATH}/${REMOTE_DIR}"
    cd "${REMOTE_PATH}/${REMOTE_DIR}"
    unzip -o "${REMOTE_PATH}/${ZIP_FILE}"
    make

    # Clean up zip file
    rm "${REMOTE_PATH}/${ZIP_FILE}"

    echo "Transfer complete!"
    cd 
EOF

# Clean up local zip file
rm "$ZIP_FILE"
echo "Local cleanup done."
