# Running as a remote device

This allows you to run the code which interfaces directly with the xum1541 on a machine close to the xum1541 (and physically attached), but run the remainder of the code on a different machine.

```
# Update your install
sudo apt update && sudo apt -y upgrade

# Install git
sudo apt -y install git

# Install Rust
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
. "$HOME/.cargo/env"

# Get the xum1541 source code
git clone https://github.com/piersfinlayson/xum1541

# Build and install the xum1541 udev rules file
./install.sh

# Set it up as a service
./remote-setup.sh
```

Then run the client (rs1541, rs1541fs, etc) in remote mode, pointing it at the remote node's IP address.
