# Linux

## Installation

To install Ubuntu Linux, follow the official documentation: https://ubuntu.com/tutorials/install-ubuntu-desktop#1-overview

### VirtualBox Installation (VB)

If you don’t have a spare machine for a clean Linux installation or cannot dual boot, consider installing Linux in a virtual machine.

VirtualBox is an example of a virtual machine hypervisor that allows you to run Linux on a different host OS.

Install VirtualBox following the instructions for your operating system:
- Windows and macOS: https://www.virtualbox.org/wiki/Downloads
- Linux: the process depends on your distribution and package manager. For Debian/Ubuntu:
  - In a terminal: `sudo apt install virtualbox`
  - Launch VirtualBox by running `virtualbox` or from your applications menu.

To install Ubuntu inside the virtual machine, follow: https://ubuntu.com/tutorials/how-to-run-ubuntu-desktop-on-a-virtual-machine-using-virtualbox#1-overview

## CLI (Command Line Interface)

Consider this chapter a quick guide for working with the Linux terminal.

You don’t need to memorize every command and parameter; be familiar with the basics and know how to look up usage when needed.

Helpful cheat sheet: https://assets.ubuntu.com/v1/2950553c-OpenStack%20cheat%20sheet%20-%20revised%20v3.pdf

### Command
Explanation of function
```
Example usage      ...      Explanation
```

### ls — list
Displays files and directories in the current location.
```bash
ls
ls -la      # lists all files, including hidden ones, with details
```

### cd — change directory
Changes the current directory.
```bash
cd my_directory      # moves into the directory named "my_directory"
cd ~                 # goes to your home directory
cd ..                # moves up one directory level
cd /                 # goes to the filesystem root
cd ../my_folder      # up one level, then into "my_folder"
cd .                 # stays in the current directory ("." means current directory)
```

### pwd — print working directory
Shows the current directory path.
```bash
pwd
```

### mkdir — make directory
Creates a new directory.
```bash
mkdir my_folder      # creates a directory named "my_folder"
```

### cp — copy
Copies files.
```bash
cp source_file destination_file             # creates a copy of "source_file" named "destination_file"
cp ../secret.txt secret_folder/supersecret.txt  # copies "secret.txt" from the parent directory to "secret_folder" as "supersecret.txt"
```

### mv — move (rename)
Originally moved files; today also commonly used to rename files.
```bash
mv old_name.txt new_name.html      # renames "old_name.txt" to "new_name.html"
```

### rm — remove
Deletes files or directories.
```bash
rm old_file.txt      # deletes the file "old_file.txt"
rm -r my_folder      # deletes a directory and its contents (recursive)
```

### chmod — change mode
Changes file access permissions.
```bash
chmod 777 /dev/ttyUSB0      # grants all users access to USB port 0 (example)
```

### sudo — run as administrator
Executes a command with administrator (root) privileges. Commonly used to modify system files.
```bash
sudo mkdir /etc/config      # creates a "config" directory in "/etc"
sudo rm -r /                # DANGEROUS: recursively deletes the root directory (destroys the system)
```

### cat — Concatenate file(s) to standard output
Prints file contents to the terminal.
```bash
cat ~/my_config_file.txt
```

### man — manual
Displays the manual for a program.
```bash
man ls
```

### Linux Distributions

Linux refers to the operating system kernel, maintained by Linus Torvalds and community contributors.

Above the kernel is a layer of package management, desktop environments, and supporting software. A Linux “distribution” bundles these components and is provided by a specific organization or vendor.

Common distributions:
- Debian — Very widespread.
- Ubuntu — Based on Debian; popular for desktops.
- Linux Mint — Based on Ubuntu; Windows-like GUI.
- Raspberry Pi OS (formerly Raspbian) — Debian-based for Raspberry Pi.
- Arch Linux — For advanced users; highly customizable.
- Fedora — A popular alternative to Debian-based systems.
- elementary OS — Minimalist and fast; good for low-spec machines.
- …and many more.

## Essential Programs

### apt
Debian/Ubuntu package manager. Software is installed from trusted repositories.

Administrator privileges are required to install software.

Example: install Git
```bash
sudo apt update
sudo apt install git
```

### nano
A simple text editor similar to Notepad.
- Ctrl+X — Exit (prompts to save changes)

### vim
A powerful text editor with a steeper learning curve. It can be much faster than nano once learned. Consider a beginner tutorial before using.

If you open vim by accident, exit with Shift+Z+Z (hold Shift and press Z twice).

### mc
Midnight Commander — a text-based file manager reminiscent of MS-DOS.
- F10 — Exit

### curl
Command-line tool for transferring data over various protocols. Often used for HTTP requests or downloads.

### wget
Downloads files from the internet.
Example: download the latest WordPress release
```bash
wget https://wordpress.org/latest.zip
```

## Final Words
If you’re new to Linux, don’t be afraid to experiment. Ideally, use a VirtualBox VM and create a snapshot/backup. If you break the system, restore the snapshot and continue working.