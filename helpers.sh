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