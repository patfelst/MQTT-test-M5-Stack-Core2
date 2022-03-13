#!/bin/sh
# Ensure make executable with chmod +x "backup to dropbox.bash"
# Run by double click this .command file from mac finder
# Command backs up the folder to dropbox, excluding the ".pio" hidden directory which contains "libdeps" (library dependencies) and "build" (compiled binaries)
# Use --dry-run for a preview of what rsync will do
CURRENT_DIR=`dirname "$0"`
# rsync --dry-run --verbose --archive --update --exclude '.pio' --exclude '.DS_Store' "$CURRENT_DIR" "/Users/patrickfelstead/Dropbox/Patrick's Files/Electronic Design/PlatformIO"
rsync --verbose --archive --update --exclude '.pio' --exclude '.DS_Store' "$CURRENT_DIR" "/Users/patrickfelstead/Dropbox/Patrick's Files/Electronic Design/PlatformIO"