#!/usr/bin/env bash
set -e  # exit if any command fails

echo ">>> Adding robotpkg repository..."
sudo sh -c "echo 'deb [arch=amd64] http://robotpkg.openrobots.org/packages/debian/pub $(lsb_release -cs) robotpkg' > /etc/apt/sources.list.d/robotpkg.list"

echo ">>> Adding robotpkg GPG key..."
sudo apt-key adv --fetch-keys http://robotpkg.openrobots.org/packages/debian/robotpkg.key

echo ">>> Updating apt..."
sudo apt update

echo ">>> Installing Pinocchio (Python 3.10 bindings)..."
sudo apt install -y robotpkg-py310-pinocchio

# Add environment setup to bashrc if not already present
BASHRC="$HOME/.bashrc"

if ! grep -q "pinocchio/robotpkg setup" "$BASHRC"; then
    echo ">>> Adding environment setup to $BASHRC"
    cat >> "$BASHRC" <<'EOF'
# >>> pinocchio/robotpkg setup >>>
export PATH=/opt/openrobots/bin:$PATH
export PKG_CONFIG_PATH=/opt/openrobots/lib/pkgconfig:$PKG_CONFIG_PATH
export LD_LIBRARY_PATH=/opt/openrobots/lib:$LD_LIBRARY_PATH
export PYTHONPATH=/opt/openrobots/lib/python3.10/site-packages:$PYTHONPATH
# <<< pinocchio/robotpkg setup <<<
EOF
fi

echo ">>> Done. Restart your shell or run 'source ~/.bashrc' to apply changes."
