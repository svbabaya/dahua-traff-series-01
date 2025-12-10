#!/bin/bash

# Script to safely delete all .o files in directory and subdirectories
# Excludes files in any directory named 'opencv'

set -e  # Exit on error

TARGET_DIR="${1:-.}"  # Use provided directory or current directory

echo "Searching for .o files in $TARGET_DIR and subdirectories (excluding opencv directories)..."

# First, count the files (excluding opencv)
file_count=$(find "$TARGET_DIR" -type f -name "*.o" ! -path "*/opencv/*" | wc -l)

if [ "$file_count" -eq 0 ]; then
    echo "No .o files found (excluding opencv)."
    exit 0
fi

# Show what will be deleted
echo "Found $file_count .o file(s) (excluding opencv):"
find "$TARGET_DIR" -type f -name "*.o" ! -path "*/opencv/*" | head -20

if [ "$file_count" -gt 20 ]; then
    echo "... and $((file_count - 20)) more"
fi

# Ask for confirmation
read -p "Delete these $file_count file(s)? (y/N): " -n 1 -r
echo

if [[ $REPLY =~ ^[Yy]$ ]]; then
    echo "Deleting files (excluding opencv)..."
    find "$TARGET_DIR" -type f -name "*.o" ! -path "*/opencv/*" -delete
    echo "Done. $file_count file(s) deleted."
else
    echo "Operation cancelled."
    exit 1
fi

