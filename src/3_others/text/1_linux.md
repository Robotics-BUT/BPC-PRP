# Linux

## Installation

To install Linux Ubuntu, please follow the [official documentation](https://ubuntu.com/tutorials/install-ubuntu-desktop#1-overview).

### VirtualBox Installation (VB)

In case you have no physical computer available for clean Linux installation nor the possibility for a dual boot, consider installing the Linux into the virtual machine.

The VirtualBox is and example of the virtual machine engine that allows you to run Linux on a different host system.

Install VirtualBox following the instructions for your respective operating system.

Steps for [Windows a Mac](https://www.virtualbox.org/wiki/Downloads).

For Linux, the installation process depends on your distribution and package management system. On Debian, use the command sudo apt install virtualbox. After installation, you can launch VirtualBox either from the terminal by typing virtualbox or by clicking its icon in the list of installed programs.

To install the Ubuntu Linux into the virtual machine, please follow the [official documentation](https://ubuntu.com/tutorials/how-to-run-ubuntu-desktop-on-a-virtual-machine-using-virtualbox#1-overview)


## CLI (Command Line Interface)

Consider this chapter as a guide for working in the Linux operating system.

You don't need to memorize all the commands and their parameters, but you should be familiar with them and able to look up their 

For more details, please find the detail [cheat sheet](https://assets.ubuntu.com/v1/2950553c-OpenStack%20cheat%20sheet%20-%20revised%20v3.pdf?_gl=1*112oq19*_gcl_au*MTE5NjAyMjU0Ni4xNzM1OTQyNzI4&_ga=2.153041082.2070240710.1735942727-373166696.1735942726) at official web site

### Command
Explanation of function
```
Example usage      ...      Explanation
```

### ls - (list)
Displays all files and directories (a directory is also a type of file) in the current point of the file system.
```
ls
ls -la      ...      lists all files, including hidden ones, and provides detailed information
```

### cd - (change directory)
Changes the current directory.
```
cd my_directory      ...      moves into the directory named "my_directory"
cd ~                 ...      returns to the home directory (referred to as "home" in Linux)
cd ..                ...      moves up one directory level
cd /                 ...      returns to the root of the file system (called "root" in Linux)
cd ../my_folder      ...      moves up one directory level and then into the "my_folder" directory
cd .                 ...      stays in the current directory. Essentially does nothing, illustrating the use of "." for the current directory.
```

### pwd - print working directory
Displays the current position in the file system.
```
pwd
```

### mkdir - (make directory)
Creates a new directory.
```
mkdir my_folder      ...      creates a new directory named "my_folder"
```

### cp - (copy
Copies a file.
```
cp source_file destination_file                    ...      creates a new copy of "source_file" named "destination_file"
cp ../secret.txt secret_folder/supersecret.txt      ...      takes the "secret.txt" file located one directory up and copies it to the "secret_folder". The copy will be named "supersecret.txt".
```

### mv - (move)
Originally used for moving files, but is now primarily used to rename files.
```
mv old_name.txt new_name.html      ...      renames the file "old_name.txt" to "new_name.html"
```

### rm - (remove)
Deletes a file or directory.
```
rm old_file.txt      ...      deletes the file "old_file.txt"
rm -r my_folder      ...      deletes a directory. The recursive modifier (-r) must always be used to delete a directory, as it specifies that the directory's contents should also be deleted.
```

### chmod - (change mode)
Changes file access permissions.
```
chmod 777 /dev/ttyUSB0      ...      grants all users access to USB port 0. For details on file system permissions, see [7].
```

### sudo
A meta command. Operations executed with this command are performed with administrator privileges. Commonly used for modifying system files.
```
sudo mkdir /etc/config      ...      creates a "config" directory in the system folder "/etc".
sudo rm -r /                ...      recursively deletes the root directory (essentially wipes the entire disk, including the OS).
```

### cat - (Concatenate FILE(s) to standard output)
Displays the contents of a file in the terminal.
```
cat ~/my_config_file.txt      ...      prints the contents of the specified file in the terminal
```

### man - (manual) referenční manuál operačního systému
Quick help if you forget how to use a specific program.
```
man ls      ...      prints the manual for the "ls" program in the terminal
```

### Linux Distributions

When we talk about Linux, we refer to the kernel of the operating system, which is managed by an authority (its creator, Linus Torvalds) that ensures the integrity of all code integrated into the OS kernel.

Above the operating system kernel lies an additional layer of package management systems, graphical interfaces, and other supporting software. A "distribution" in Linux refers to a bundle of these supporting software components, provided and guaranteed by a specific legal entity (commercial company, organization, etc.).

Commonly used distributions:

 - Debian - The most widespread Linux distribution.
 - Ubuntu - A derivative of Debian. The most popular distribution for home workstations.
 - Mint - A derivative of Ubuntu. Its GUI is similar to Windows.
 - RaspberryOS (formerly Raspbian) - A Debian derivative for Raspberry Pi.
 - Arch Linux - A distribution aimed at advanced users, offering great freedom in system configuration.
 - Fedora - An alternative to Debian.
 - ElementaryOS - A minimalist and fast distribution. Suitable for low-performance computers.
 - ... and many more.


## Essential Programs

### apt

This is Debian's package management system. On Linux, we typically install programs by downloading them from a public repository, which is usually a verified and secure server.

You always need administrative privileges to install programs.

Example of installing Git:
```
sudo apt update
sudo apt install git
```
This means: "With administrative privileges, run the apt program to update repository records" and "With administrative privileges, run the apt program to install Git."

### nano
A text editor similar to Notepad.
- Ctrl + X - Exits the program. It will ask if you want to save changes.

### vim
A professional text editor. However, its operation is somewhat complex and requires an understanding of several principles. Working with vim is significantly faster than with nano. Before using it, it is recommended to go through any "vim noob tutorial" on YouTube.

If you accidentally open vim, you can close it with the key combination **Shift + Z + Z** (hold Shift and press 'Z' twice).

### mc
**Midnight Commander** - A graphical interface for navigating the file system, resembling MS-DOS.
- Exits with the **F10** key.

### curl
A command-line tool for transferring data using various protocols. Curl is often used for HTTP communication, installing programs, or downloading files.

### wget
A program for downloading files from the internet.
Example of downloading the latest WordPress release:
```
wget https://wordpress.org/latest.zip
```

## Final Words
If you're new to Linux, don't be afraid to experiment. Ideally, install the system in VirtualBox and create a backup of the virtual disk. If you manage to mess up the system, simply restore the backup and continue working.