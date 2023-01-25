#!/bin/bash

# Get bashrc file path to variable
BASHRC="${HOME}/.bashrc"

# Check if bashrc exists, if it doesn't, create it
if [ ! -f "$BASHRC" ]; then
  touch "$BASHRC"
fi

# Helper functions
write_bashrc() {
  LINE="$1"
  grep -qF -- "$LINE" "$BASHRC" || echo "$LINE" >> "$BASHRC"
}

write_and_source_bashrc() {
  LINE="$1"
  write_bashrc "$LINE"
  source "$BASHRC"
}

require_file() {
  FILE="$1"
  if [ ! -f "$FILE" ]; then
    echo "$2"
    exit 1
  fi
}

require_folder() {
  FOLDER="$1"
  if [ ! -d "$FOLDER" ]; then
    echo "$2"
    exit 1
  fi
}

require_or_create_file() {
  FILE="$1"
  if [ ! -f "$FILE" ]; then
    touch "$FILE"
  fi
}

require_or_create_folder() {
  FOLDER="$1"
  if [ ! -d "$FOLDER" ]; then
    mkdir "$FOLDER"
  fi
}