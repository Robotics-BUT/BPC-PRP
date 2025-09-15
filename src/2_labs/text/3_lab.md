# Lab 3 - Git & C++ Project Template

Responsible: Ing. Jakub Minařík

## Git (1h 30min)

First, read the [Git tutorial](../../4_others/text/3_git.md) to get familiar with the workflow and commands.

### Exercise

### Sign On 

Select one of the following free Git services and register.
 - [GitHub](https://github.com/)
 - [GitLab](https://about.gitlab.com/)
 - [Bitbucket](https://bitbucket.org/product/)

This server will serve as your **"origin"** (remote repository) for the rest of the **BPC-PRP** course.

The instructors will have access to all your repositories, including their history, and can monitor your progress, including who, when, and how frequently commits were made.

Create a repository on the server to maintain your code throughout the course.

---

### Cloning the repository in the lab

#### HTTPS - GitHub Token  

When cloning a repository via HTTPS, you cannot push changes using your username and password. Instead, you must use a generated GitHub token.  

To generate a token, go to **Profile picture (top-right corner)** > **Settings** > **Developer Settings** > **Personal Access Tokens** > **Tokens (classic)** or click [here](https://github.com/settings/tokens). Your generated token will be shown only once, after which you can use it as a password when pushing changes via HTTPS until the token expires.  

#### SSH - Setting a Different Key  

You can generate an SSH key using the `ssh-keygen` command. It will prompt you for the file location/name and then for a passphrase. For lab use, set a passphrase. The default location is `~/.ssh`.  

When cloning a repository via SSH in the lab, you may encounter a problem with Git using the wrong SSH key.  
You'll need to configure Git to use your generated key:  
```bash
git config core.sshCommand "ssh -i ~/.ssh/<your_key>"
```
In this command, `<your_key>` refers to the private part of your generated key.  

On GitHub, you can add the **public** part of your key to either a specific repository or your entire account.  

- **To add a key to a project (repository level):**  
  Go to **Project** > **Settings** > **Deploy keys** > **Add deploy key**, then check **Allow write access** if needed.  

- **To add a key to your GitHub account (global access):**  
  Go to **Profile picture (top-right corner)** > **Settings** > **SSH and GPG keys** > **New SSH key**.  



### Team Exercise

As a team, complete the following steps:

1. One team member creates a repository on the server.
2. All team members clone the repository to their local machines.
3. One team member creates a "Hello, World!" program locally, commits it, and pushes it to the origin.
4. The rest of the team pulls the changes to their local repositories.
5. Two team members intentionally create a conflict by modifying the same line of code simultaneously and attempting to push their changes to the server. The second member to push will receive an error from Git indicating a conflict.
6. The team member who encounters the conflict resolves it and pushes the corrected version to the origin.
7. All team members pull the updated version of the repository. Each member then creates their own `.h` file containing a function that prints their name. Everyone pushes their changes to the server.
8. One team member pulls the newly created `.h` files and modifies the "Hello, World!" program to use all the newly created code. The changes are then pushed to the origin.
9. All team members pull the latest state of the repository.

## C++ Project Template (30 min)

Now it is time to create your main project for this course.

1. Create a project on the web page of your Git service.
2. Clone the project to your local machine.
3. Create the following project structure

```
/bpc-prp-project-team-x
 |--docs
 | \--placeholder
 |--README.md
 |--CMakeLists.txt
 |--.gitignore
 |--include
 | \--<project_name>
 |   \--lib.hpp
 \--src
   |--lib.cpp
   \--main.cpp
```

4. Fill all required files
 - README.md: a brief description and how to use your project.
 - The `docs` folder will be used later. For now, just create a file named `placeholder`.
 - Write some basic code into the `cpp` and `hpp` files.
 - Fill the `.gitignore` file to keep build artifacts and IDE files out of the repository.

```gitignore
# Ignore build directories
/build/
/cmake-build-debug/
/cmake-build-release/

# Ignore CMake-generated files
CMakeFiles/
CMakeCache.txt
cmake_install.cmake
Makefile

# Ignore IDE-specific files (CLion and JetBrains)
.idea/
*.iml
```
5. Commit and push your project to the server and share it with other members of the team.


