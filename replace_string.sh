#!/bin/bash

# Check if the correct number of arguments is provided
if [ "$#" -ne 3 ]; then
    echo "Usage: $0 <file_path> <search_string> <replace_string>"
    exit 1
fi

# Assign arguments to variables
file_path="$1"
search_string="$2"
replace_string="$3"

# Check if the file exists
if [ ! -f "$file_path" ]; then
    echo "Error: File not found at '$file_path'"
    exit 1
fi

# Perform the string replacement
sed -i "s/${search_string}/${replace_string}/g" "$file_path"

# Notify the user of success
echo "Replaced all occurrences of '${search_string}' with '${replace_string}' in '$file_path'."

